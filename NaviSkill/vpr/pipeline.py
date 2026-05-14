"""VPR Pipeline V1: Global descriptors + FAISS + optional re-ranking.

This is the main entry point. It orchestrates the pipeline:
  Stage 1: Extract global descriptors (MegaLoc or SALAD-indoor)
  Stage 2: kNN retrieval (FAISS)
  Stage 3: Geometric re-ranking (SuperPoint + LightGlue) [optional]
  Stage 4: Color histogram re-scoring [optional]

Usage:
    python pipeline.py \
        --method=salad-indoor \
        --database_folder=path/to/database \
        --queries_folder=path/to/queries \
        --color_rerank --color_weight=5.0 --num_preds_to_save=3
"""

import argparse
import sys
from datetime import datetime
from pathlib import Path

import faiss
import numpy as np
import torch
from loguru import logger
from torch.utils.data import DataLoader
from torch.utils.data.dataset import Subset
from tqdm import tqdm

from dataset import VPRDataset
from model_loader import get_model


def parse_args():
    parser = argparse.ArgumentParser(
        description="VPR Pipeline V1: FAISS retrieval with optional re-ranking",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    parser.add_argument("--database_folder", type=str, required=True, help="Path to database images")
    parser.add_argument("--queries_folder", type=str, required=True, help="Path to query images")
    parser.add_argument("--method", type=str, default="megaloc",
                        choices=["megaloc", "salad-indoor"],
                        help="VPR model: megaloc (outdoor/geometry) or salad-indoor (indoor/appearance)")
    parser.add_argument("--device", type=str, default="cuda", choices=["cuda", "cpu"])
    parser.add_argument("--batch_size", type=int, default=4)
    parser.add_argument("--num_workers", type=int, default=4)
    parser.add_argument("--log_dir", type=str, default="default")

    parser.add_argument("--no_labels", action="store_true",
                        help="Skip ground-truth recall (use when poses aren't available)")
    parser.add_argument("--positive_dist_threshold", type=int, default=25,
                        help="Distance (m) for a prediction to count as correct")
    parser.add_argument("--recall_values", type=int, nargs="+", default=[1, 5, 10, 20])

    parser.add_argument("--rerank", action="store_true",
                        help="Enable geometric re-ranking with SuperPoint+LightGlue")
    parser.add_argument("--rerank_top_k", type=int, default=20,
                        help="Number of FAISS candidates to re-rank per query")

    parser.add_argument("--color_rerank", action="store_true",
                        help="Re-score candidates using HSV color histogram similarity")
    parser.add_argument("--color_weight", type=float, default=0.5,
                        help="Weight for color similarity: score = geo * (1 + alpha * color_sim)")

    parser.add_argument("--num_preds_to_save", type=int, default=0,
                        help="Save top-N prediction visualizations per query (0=disabled)")
    parser.add_argument("--save_only_wrong_preds", action="store_true")
    parser.add_argument("--save_descriptors", action="store_true")
    parser.add_argument("--image_size", type=int, nargs="+", default=None)

    args = parser.parse_args()
    args.use_labels = not args.no_labels

    if args.rerank and max(args.recall_values) < args.rerank_top_k:
        args.recall_values.append(args.rerank_top_k)
    if args.num_preds_to_save > 0 and max(args.recall_values) < args.num_preds_to_save:
        args.recall_values.append(args.num_preds_to_save)

    return args


def run_pipeline(args):
    start_time = datetime.now()

    # --- Logging ---
    logger.remove()
    log_dir = Path("logs") / args.log_dir / start_time.strftime("%Y-%m-%d_%H-%M-%S")
    logger.add(sys.stdout, colorize=True, format="<green>{time:%Y-%m-%d %H:%M:%S}</green> {message}", level="INFO")
    logger.add(log_dir / "info.log", format="<green>{time:%Y-%m-%d %H:%M:%S}</green> {message}", level="INFO")
    logger.add(log_dir / "debug.log", level="DEBUG")
    logger.info(f"Arguments: {args}")
    logger.info(f"Output: {log_dir}")

    # --- Stage 1: Load model and extract descriptors ---
    logger.info("Stage 1: Loading model and extracting descriptors")
    model, descriptor_dim = get_model(args.method, args.device)

    dataset = VPRDataset(
        args.database_folder, args.queries_folder,
        positive_dist_threshold=args.positive_dist_threshold,
        image_size=args.image_size,
        use_labels=args.use_labels,
    )
    logger.info(f"Dataset: {dataset}")

    all_descriptors = np.empty((len(dataset), descriptor_dim), dtype="float32")

    with torch.inference_mode():
        db_subset = Subset(dataset, list(range(dataset.num_database)))
        db_loader = DataLoader(dataset=db_subset, num_workers=args.num_workers, batch_size=args.batch_size)
        for images, indices in tqdm(db_loader, desc="Database descriptors"):
            descriptors = model(images.to(args.device)).cpu().numpy()
            all_descriptors[indices.numpy(), :] = descriptors

        q_subset = Subset(dataset, list(range(dataset.num_database, dataset.num_database + dataset.num_queries)))
        q_loader = DataLoader(dataset=q_subset, num_workers=args.num_workers, batch_size=1)
        for images, indices in tqdm(q_loader, desc="Query descriptors"):
            descriptors = model(images.to(args.device)).cpu().numpy()
            all_descriptors[indices.numpy(), :] = descriptors

    queries_descriptors = all_descriptors[dataset.num_database:]
    database_descriptors = all_descriptors[:dataset.num_database]

    if args.save_descriptors:
        np.save(log_dir / "queries_descriptors.npy", queries_descriptors)
        np.save(log_dir / "database_descriptors.npy", database_descriptors)

    # --- Stage 2: FAISS retrieval ---
    logger.info("Stage 2: FAISS kNN retrieval")
    faiss_index = faiss.IndexFlatL2(descriptor_dim)
    faiss_index.add(database_descriptors)
    del database_descriptors, all_descriptors

    search_k = max(args.recall_values)
    faiss_distances, predictions = faiss_index.search(queries_descriptors, search_k)

    # --- Stage 3: Geometric re-ranking (optional) ---
    rerank_scores = None
    if args.rerank:
        from rerank import GeometricReranker

        logger.info(f"Stage 3: Re-ranking top-{args.rerank_top_k} with SuperPoint+LightGlue")
        reranker = GeometricReranker(device=args.device)
        predictions, rerank_scores = reranker.rerank_all(predictions, dataset)

        top1 = rerank_scores[:, 0]
        logger.info(
            f"Re-ranking complete. Top-1 inlier stats: "
            f"mean={top1.mean():.1f}, median={np.median(top1):.0f}, "
            f"min={top1.min()}, max={top1.max()}, "
            f"zero_inlier_queries={int((top1 == 0).sum())}/{len(top1)}"
        )
        np.save(log_dir / "rerank_scores.npy", rerank_scores)

    # --- Stage 4: Color histogram re-scoring (optional) ---
    if args.color_rerank:
        from color_rerank import ColorReranker

        logger.info(f"Stage 4: Color re-scoring with alpha={args.color_weight}")
        color_reranker = ColorReranker(alpha=args.color_weight)

        if rerank_scores is not None:
            base_scores = rerank_scores.astype(np.float64)
        else:
            max_dist = faiss_distances.max() + 1.0
            base_scores = (max_dist - faiss_distances).astype(np.float64)

        predictions, rerank_scores = color_reranker.rescore_all(
            predictions, base_scores, dataset,
        )

        top1 = rerank_scores[:, 0]
        logger.info(
            f"Color re-scoring complete. Top-1 combined stats: "
            f"mean={top1.mean():.1f}, median={np.median(top1):.0f}, "
            f"min={top1.min():.1f}, max={top1.max():.1f}"
        )
        np.save(log_dir / "color_rerank_scores.npy", rerank_scores)

    np.save(log_dir / "predictions.npy", predictions)

    # --- Recall evaluation ---
    if args.use_labels:
        positives_per_query = dataset.get_positives()
        recalls = np.zeros(len(args.recall_values))
        for query_index, preds in enumerate(predictions):
            for i, n in enumerate(args.recall_values):
                if np.any(np.isin(preds[:n], positives_per_query[query_index])):
                    recalls[i:] += 1
                    break
        recalls = recalls / dataset.num_queries * 100
        recalls_str = ", ".join([f"R@{v}: {r:.1f}" for v, r in zip(args.recall_values, recalls)])
        logger.info(recalls_str)

    # --- Save predictions ---
    if args.num_preds_to_save > 0:
        from visualizations import save_preds

        logger.info("Saving prediction visualizations")
        preds_to_save = predictions[:, :args.num_preds_to_save]
        scores_to_save = rerank_scores[:, :args.num_preds_to_save] if rerank_scores is not None else None
        save_preds(
            preds_to_save, dataset, log_dir,
            args.save_only_wrong_preds, args.use_labels,
            confidence_scores=scores_to_save,
        )

    logger.info(f"Done. Results saved to {log_dir}")
    return predictions, rerank_scores


if __name__ == "__main__":
    args = parse_args()
    run_pipeline(args)
