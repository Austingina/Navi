# Copyright (c) Meta Platforms, Inc. and affiliates. All Rights Reserved
"""Macro precision (per-NP averaged) between a labeler ground-truth
``annotations.json`` and a SAM 3 inference ``annotations.json`` /
``predictions.json``.

Both files share the COCO-NP-pair schema written by
:mod:`sam3.labeler.compact` and
:class:`sam3.labeler.annotations.InferenceAnnotationBuilder`. Boxes are
normalized xywh in ``[0, 1]``, and pairs are joined by
``(file_name, text_input)`` because each run numbers ``images[].id``
from 1 (i.e. image ids are file-local and not a valid join key).

Matching follows COCO conventions:

- IoU is computed on bounding boxes (no mask IoU; the inference scripts
  do not emit RLE segmentations).
- Predictions are sorted by ``score`` descending and greedily matched
  to unmatched GT boxes with IoU above a threshold; the first
  remaining GT box above the threshold wins.
- Precision per NP = ``sum(TP) / sum(TP + FP)`` aggregated across all
  images of that NP.
- **Macro precision** = unweighted mean of precision over NPs that have
  at least one prediction (NPs with zero predictions have an undefined
  precision and are flagged ``n/a`` rather than coerced to 0).

CLI::

    python -m sam3.labeler.evaluate \\
        --gt   sam3/labels/annotations.json \\
        --pred sam3/rainbow_surrounding_outputs/temp/predictions.json \\
        --iou  0.5

Pass ``--iou`` multiple times to see several thresholds side by side
(e.g. ``--iou 0.5 --iou 0.75``). ``--score FLOAT`` drops low-confidence
predictions before matching. ``--include_pred_only_pairs`` counts
predictions for ``(file, NP)`` pairs that exist only in the prediction
file as pure false positives (open-world evaluation). ``--json`` dumps
the full per-NP table as machine-readable JSON instead of the ASCII
report.
"""

from __future__ import annotations

import argparse
import json
import logging
import sys
from collections import defaultdict
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------- #
# I/O
# ---------------------------------------------------------------------- #
def _load_coco_np(
    path: str,
) -> Tuple[
    Dict[Tuple[str, str], List[Dict[str, Any]]], List[Dict[str, Any]]
]:
    """Read a COCO-NP-pair JSON and bucket annotations by ``(file_name, NP)``.

    Returns ``(by_pair, images)`` where ``by_pair[(file, np)]`` is the
    list of annotation dicts whose ``image_id`` references that pair.
    """
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    images = data.get("images") or []
    imgs_by_id = {int(img["id"]): img for img in images}
    by_pair: Dict[Tuple[str, str], List[Dict[str, Any]]] = defaultdict(list)
    for ann in data.get("annotations") or []:
        img = imgs_by_id.get(int(ann.get("image_id", -1)))
        if img is None:
            logger.warning(
                "annotation id=%s references unknown image_id=%s; skipping.",
                ann.get("id"),
                ann.get("image_id"),
            )
            continue
        key = (img.get("file_name", ""), img.get("text_input", ""))
        by_pair[key].append(ann)
    # Also materialize pairs that have zero annotations (e.g. negative
    # GT pairs, or pred-only NP queries that returned no detections).
    for img in images:
        key = (img.get("file_name", ""), img.get("text_input", ""))
        by_pair.setdefault(key, [])
    return by_pair, images


# ---------------------------------------------------------------------- #
# IoU on normalized xywh
# ---------------------------------------------------------------------- #
def _iou_xywh(a: Sequence[float], b: Sequence[float]) -> float:
    ax, ay, aw, ah = a
    bx, by, bw, bh = b
    ix1 = max(ax, bx)
    iy1 = max(ay, by)
    ix2 = min(ax + aw, bx + bw)
    iy2 = min(ay + ah, by + bh)
    iw = max(0.0, ix2 - ix1)
    ih = max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0.0:
        return 0.0
    union = aw * ah + bw * bh - inter
    if union <= 0:
        return 0.0
    return inter / union


# ---------------------------------------------------------------------- #
# Greedy per-pair matching
# ---------------------------------------------------------------------- #
def _match_one_pair(
    gt: Sequence[Dict[str, Any]],
    pred: Sequence[Dict[str, Any]],
    iou_thr: float,
) -> Tuple[int, int]:
    """Return ``(tp, fp)`` for one ``(file, NP)`` pair.

    Predictions are sorted by score (desc); each prediction grabs the
    unmatched GT with the highest IoU that is at least ``iou_thr``.
    """
    matched = [False] * len(gt)
    tp = fp = 0
    preds_sorted = sorted(
        pred, key=lambda p: float(p.get("score", 0.0)), reverse=True
    )
    for p in preds_sorted:
        best_idx = -1
        best_iou = iou_thr  # strict >= threshold
        for i, g in enumerate(gt):
            if matched[i]:
                continue
            iou = _iou_xywh(p.get("bbox", [0, 0, 0, 0]), g.get("bbox", [0, 0, 0, 0]))
            if iou >= best_iou:
                best_idx = i
                best_iou = iou
        if best_idx >= 0:
            matched[best_idx] = True
            tp += 1
        else:
            fp += 1
    return tp, fp


# ---------------------------------------------------------------------- #
# Evaluation driver
# ---------------------------------------------------------------------- #
def evaluate(
    gt_path: str,
    pred_path: str,
    iou_thresholds: Sequence[float] = (0.5,),
    score_threshold: float = 0.0,
    include_pred_only_pairs: bool = False,
) -> Dict[str, Any]:
    """Compute per-NP precision/recall and macro precision.

    Parameters
    ----------
    gt_path, pred_path:
        Paths to COCO-NP-pair JSONs (the labeler's
        ``annotations.json`` and a SAM 3 inference output respectively).
    iou_thresholds:
        Sequence of IoU thresholds to evaluate at; macro precision is
        reported for each.
    score_threshold:
        Drop predictions whose ``score`` is below this value before
        matching.
    include_pred_only_pairs:
        If True, also evaluate ``(file, NP)`` pairs that exist only in
        the prediction file (their predictions become pure FPs). If
        False (default), evaluation is restricted to pairs present in
        the ground truth.

    Returns
    -------
    A nested dict with per-NP counts and aggregate macro precision at
    each IoU threshold. See :func:`_format_report` for the ASCII
    rendering.
    """
    gt_by_pair, gt_images = _load_coco_np(gt_path)
    pred_by_pair, pred_images = _load_coco_np(pred_path)

    if include_pred_only_pairs:
        pair_keys = set(gt_by_pair.keys()) | set(pred_by_pair.keys())
    else:
        pair_keys = set(gt_by_pair.keys())

    if score_threshold > 0.0:
        pred_by_pair = {
            k: [p for p in v if float(p.get("score", 0.0)) >= score_threshold]
            for k, v in pred_by_pair.items()
        }

    counts: Dict[float, Dict[str, Dict[str, int]]] = {
        thr: defaultdict(lambda: {"tp": 0, "fp": 0, "gt": 0, "preds": 0})
        for thr in iou_thresholds
    }
    for key in sorted(pair_keys):
        _file_name, np_text = key
        gt = gt_by_pair.get(key, [])
        pred = pred_by_pair.get(key, [])
        for thr in iou_thresholds:
            tp, fp = _match_one_pair(gt, pred, thr)
            c = counts[thr][np_text]
            c["tp"] += tp
            c["fp"] += fp
            c["gt"] += len(gt)
            c["preds"] += len(pred)

    by_iou: Dict[float, Dict[str, Any]] = {}
    for thr in iou_thresholds:
        per_np: Dict[str, Dict[str, Any]] = {}
        precisions: List[float] = []
        for np_text, c in sorted(counts[thr].items()):
            denom_p = c["tp"] + c["fp"]
            denom_r = c["gt"]
            precision = (c["tp"] / denom_p) if denom_p > 0 else None
            recall = (c["tp"] / denom_r) if denom_r > 0 else None
            per_np[np_text] = {**c, "precision": precision, "recall": recall}
            if precision is not None:
                precisions.append(precision)
        macro = sum(precisions) / len(precisions) if precisions else None
        by_iou[thr] = {
            "per_np": per_np,
            "macro_precision": macro,
            "num_nps_with_predictions": len(precisions),
            "num_nps_total": len(per_np),
        }

    return {
        "gt_path": gt_path,
        "pred_path": pred_path,
        "n_pairs_evaluated": len(pair_keys),
        "n_pairs_gt_only": len(set(gt_by_pair.keys()) - set(pred_by_pair.keys())),
        "n_pairs_pred_only": len(set(pred_by_pair.keys()) - set(gt_by_pair.keys())),
        "score_threshold": float(score_threshold),
        "include_pred_only_pairs": bool(include_pred_only_pairs),
        "by_iou": by_iou,
    }


# ---------------------------------------------------------------------- #
# ASCII report
# ---------------------------------------------------------------------- #
def _fmt_float(x: Optional[float]) -> str:
    return "n/a" if x is None else f"{x:.4f}"


def _format_report(report: Dict[str, Any]) -> str:
    lines: List[str] = []
    lines.append(f"GT  : {report['gt_path']}")
    lines.append(f"Pred: {report['pred_path']}")
    lines.append(
        f"Pairs evaluated: {report['n_pairs_evaluated']} "
        f"(gt-only={report['n_pairs_gt_only']}, "
        f"pred-only={report['n_pairs_pred_only']}; "
        f"pred-only pairs "
        f"{'included' if report['include_pred_only_pairs'] else 'excluded'}; "
        f"score >= {report['score_threshold']:.2f})"
    )

    for thr in sorted(report["by_iou"].keys()):
        blob = report["by_iou"][thr]
        lines.append("")
        lines.append(f"=== IoU >= {thr:.2f} ===")
        header = (
            f"{'noun phrase':<30s} {'TP':>5} {'FP':>5} "
            f"{'#GT':>5} {'#Pred':>6} {'precision':>10} {'recall':>10}"
        )
        lines.append(header)
        lines.append("-" * len(header))
        for np_text, row in blob["per_np"].items():
            lines.append(
                f"{np_text[:30]:<30s} "
                f"{row['tp']:>5d} {row['fp']:>5d} "
                f"{row['gt']:>5d} {row['preds']:>6d} "
                f"{_fmt_float(row['precision']):>10s} "
                f"{_fmt_float(row['recall']):>10s}"
            )
        lines.append("-" * len(header))
        macro_s = _fmt_float(blob["macro_precision"])
        lines.append(
            f"macro precision over {blob['num_nps_with_predictions']} / "
            f"{blob['num_nps_total']} NPs with predictions: {macro_s}"
        )
    return "\n".join(lines)


# ---------------------------------------------------------------------- #
# CLI
# ---------------------------------------------------------------------- #
def _parse_args(argv: List[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        prog="python -m sam3.labeler.evaluate",
        description=(
            "Compute macro precision (and per-NP precision / recall) "
            "between a SAM 3 labeler annotations.json and a model "
            "predictions JSON in the same COCO-NP-pair schema."
        ),
    )
    p.add_argument("--gt", required=True, help="Path to ground-truth annotations.json.")
    p.add_argument(
        "--pred",
        required=True,
        help="Path to predictions JSON (same schema; produced by e.g. trt_infer).",
    )
    p.add_argument(
        "--iou",
        type=float,
        action="append",
        help=(
            "IoU threshold for a TP match. Pass multiple times for "
            "multiple thresholds (default: 0.5)."
        ),
    )
    p.add_argument(
        "--score",
        type=float,
        default=0.0,
        help="Drop predictions whose score is below this threshold before matching.",
    )
    p.add_argument(
        "--include_pred_only_pairs",
        action="store_true",
        help=(
            "Also evaluate (file, NP) pairs that exist only in the "
            "prediction file (their predictions become pure FPs)."
        ),
    )
    p.add_argument(
        "--json",
        action="store_true",
        help="Dump the full report as JSON instead of the ASCII table.",
    )
    p.add_argument(
        "-v", "--verbose", action="store_true", help="Enable INFO-level logging."
    )
    return p.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = _parse_args(argv if argv is not None else sys.argv[1:])
    logging.basicConfig(
        level=logging.INFO if args.verbose else logging.WARNING,
        format="%(levelname)s %(name)s: %(message)s",
    )
    ious = args.iou if args.iou else [0.5]
    report = evaluate(
        args.gt,
        args.pred,
        iou_thresholds=ious,
        score_threshold=args.score,
        include_pred_only_pairs=args.include_pred_only_pairs,
    )
    if args.json:
        json.dump(report, sys.stdout, indent=2)
        print()
    else:
        print(_format_report(report))
    return 0


if __name__ == "__main__":  # pragma: no cover - CLI entry point
    raise SystemExit(main())
