"""Prediction visualization utilities.

Generates side-by-side images showing query + top-N predictions,
with green/red boxes for correct/incorrect matches (when labels available)
and text files with paths and confidence scores.
"""

import numpy as np
import torch
import torchvision.transforms as tfm
from PIL import Image, ImageDraw, ImageFont, ImageOps
from tqdm import tqdm

IMG_HW = 512
TEXT_H = 175
FONTSIZE = 50
SPACE = 50


def write_labels_to_image(labels):
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", FONTSIZE)
    img = Image.new("RGB", ((IMG_HW * len(labels)) + 50 * (len(labels) - 1), TEXT_H), (1, 1, 1))
    d = ImageDraw.Draw(img)
    for i, text in enumerate(labels):
        _, _, w, h = d.textbbox((0, 0), text, font=font)
        d.text(((IMG_HW + SPACE) * i + IMG_HW // 2 - w // 2, 1), text, fill=(0, 0, 0), font=font)
    return Image.fromarray(np.array(img)[:100] * 255)


def draw_box(img, c=(0, 1, 0), thickness=20):
    img = tfm.ToTensor()(img)
    c = torch.tensor(c).type(torch.float).reshape(3, 1, 1)
    img[..., :thickness, :] = c
    img[..., -thickness:, :] = c
    img[..., :, -thickness:] = c
    img[..., :, :thickness] = c
    return tfm.ToPILImage()(img)


def build_prediction_image(images_paths, preds_correct):
    labels = ["Query"]
    for i, is_correct in enumerate(preds_correct[1:]):
        if is_correct is None:
            labels.append(f"Pred{i}")
        else:
            labels.append(f"Pred{i} - {is_correct}")

    images = [Image.open(path).convert("RGB") for path in images_paths]
    for img_idx, (img, is_correct) in enumerate(zip(images, preds_correct)):
        if is_correct is None:
            continue
        color = (0, 1, 0) if is_correct else (1, 0, 0)
        images[img_idx] = draw_box(img, color)

    resized_images = [ImageOps.pad(img, (IMG_HW, IMG_HW), color='white') for img in images]

    total_w = len(resized_images) * IMG_HW + max(0, len(resized_images) - 1) * SPACE
    concat_image = Image.new('RGB', (total_w, IMG_HW), (255, 255, 255))
    x = 0
    for img in resized_images:
        concat_image.paste(img, (x, 0))
        x += IMG_HW + SPACE

    try:
        labels_image = write_labels_to_image(labels)
        final_image = Image.fromarray(
            np.concatenate((np.array(labels_image), np.array(concat_image)), axis=0)
        )
    except OSError:
        final_image = concat_image

    return final_image


def save_preds(predictions, eval_ds, log_dir, save_only_wrong_preds=None,
               use_labels=True, confidence_scores=None):
    if use_labels:
        positives_per_query = eval_ds.get_positives()

    viz_dir = log_dir / "preds"
    viz_dir.mkdir(parents=True, exist_ok=True)

    for query_index, preds in enumerate(tqdm(predictions, desc=f"Saving preds in {viz_dir}")):
        query_path = eval_ds.queries_paths[query_index]
        list_of_images_paths = [query_path]
        preds_correct = [None]

        for pred in preds:
            pred_path = eval_ds.database_paths[pred]
            list_of_images_paths.append(pred_path)
            if use_labels:
                is_correct = pred in positives_per_query[query_index]
            else:
                is_correct = None
            preds_correct.append(is_correct)

        if save_only_wrong_preds and preds_correct[1]:
            continue

        prediction_image = build_prediction_image(list_of_images_paths, preds_correct)
        prediction_image.save(viz_dir / f"{query_index:03d}.jpg")

        file_content = [
            "Query path:", query_path, "",
            "Predictions paths:", "\n".join(list_of_images_paths[1:]), "",
        ]
        if confidence_scores is not None:
            file_content.extend([
                "Confidence (inlier count):",
                ", ".join(str(int(s)) for s in confidence_scores[query_index]), "",
            ])
        if use_labels:
            positives_paths = [eval_ds.database_paths[idx] for idx in positives_per_query[query_index]]
            file_content.extend(["Positives paths:", "\n".join(positives_paths), ""])

        with open(viz_dir / f"{query_index:03d}.txt", "w") as f:
            f.write("\n".join(file_content))
