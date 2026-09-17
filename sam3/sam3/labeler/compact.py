# Copyright (c) Meta Platforms, Inc. and affiliates. All Rights Reserved
"""Merge per-pair shards into a canonical ``annotations.json``.

The labeler writes one self-contained JSON shard per image-NP pair
under ``<output_dir>/pairs/`` (see :class:`sam3.labeler.annotations.\
AnnotationStore`). SAM 3 eval and training, however, expect a single
``annotations.json`` whose IDs are monotonic. This module provides the
deterministic merge step plus a one-shot ``--split`` migration for
directories whose data still lives in the old monolithic format.

CLI::

    # Build <output_dir>/annotations.json from <output_dir>/pairs/*.json
    python -m sam3.labeler.compact <output_dir>

    # Optional: write to an explicit path instead.
    python -m sam3.labeler.compact <output_dir> --out other.json

    # Migrate an existing monolithic annotations.json into shards.
    python -m sam3.labeler.compact <output_dir> --split
    # (or point at an arbitrary file)
    python -m sam3.labeler.compact <output_dir> --split --in legacy.json

Determinism: ``compact_pairs`` sorts shard filenames with ``sorted()``
before merging, so re-running it on the same directory produces a
byte-identical ``annotations.json``.
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import sys
import tempfile
from typing import Any, Dict, List, Optional

from sam3.labeler.annotations import (
    _DEFAULT_CATEGORIES,
    _DEFAULT_INFO,
    _slugify,
)

logger = logging.getLogger(__name__)


def _atomic_dump_json(path: str, data: Dict[str, Any]) -> None:
    """Atomically write ``data`` as pretty-printed JSON to ``path``."""
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    fd, tmp = tempfile.mkstemp(
        prefix=".compact.",
        suffix=".json.tmp",
        dir=os.path.dirname(path) or ".",
    )
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    except Exception:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


def compact_pairs(
    output_dir: str, out_path: Optional[str] = None
) -> Dict[str, Any]:
    """Merge ``<output_dir>/pairs/*.json`` into a single COCO-NP-pair file.

    Shards are read in sorted-filename order so the result is
    deterministic. ``images[].id`` is renumbered monotonically starting
    at 1; each annotation's ``image_id`` is rewritten to the new image
    id and ``annotations[].id`` is reassigned across the whole
    directory.

    A malformed or unreadable shard is logged and skipped rather than
    aborting the whole compaction.

    Parameters
    ----------
    output_dir:
        Directory holding ``pairs/``.
    out_path:
        Where to write the merged JSON. Defaults to
        ``<output_dir>/annotations.json``.

    Returns
    -------
    The merged dict that was written (useful for tests / chained
    pipelines).
    """
    output_dir = os.path.abspath(output_dir)
    pairs_dir = os.path.join(output_dir, "pairs")
    if out_path is None:
        out_path = os.path.join(output_dir, "annotations.json")

    merged: Dict[str, Any] = {
        "info": dict(_DEFAULT_INFO),
        "images": [],
        "annotations": [],
        "categories": list(_DEFAULT_CATEGORIES),
    }

    if not os.path.isdir(pairs_dir):
        logger.warning(
            "No pairs/ directory at %s; writing an empty annotations.json.",
            pairs_dir,
        )
        _atomic_dump_json(out_path, merged)
        return merged

    next_img_id = 1
    next_ann_id = 1
    shard_names = sorted(
        n for n in os.listdir(pairs_dir) if n.lower().endswith(".json")
    )

    for name in shard_names:
        shard_path = os.path.join(pairs_dir, name)
        try:
            with open(shard_path, "r", encoding="utf-8") as f:
                shard = json.load(f)
        except (OSError, json.JSONDecodeError) as e:
            logger.warning("Skipping malformed shard %s: %s", shard_path, e)
            continue

        shard_images = shard.get("images") or []
        shard_anns = shard.get("annotations") or []
        if not shard_images:
            logger.warning(
                "Shard %s has no images[] entry; skipping.", shard_path
            )
            continue

        # A labeler shard always contains exactly one image entry, but
        # we tolerate well-formed multi-image shards too (e.g. ones that
        # were produced by ``split_annotations`` and then concatenated
        # by hand). For each image in the shard, rewrite IDs and
        # forward its annotations.
        local_to_canonical: Dict[int, int] = {}
        for img in shard_images:
            local_id = int(img.get("id", 1))
            canonical_id = next_img_id
            local_to_canonical[local_id] = canonical_id
            new_img = dict(img)
            new_img["id"] = canonical_id
            merged["images"].append(new_img)
            next_img_id += 1

        for ann in shard_anns:
            local_img_id = int(ann.get("image_id", 1))
            canonical_img_id = local_to_canonical.get(local_img_id)
            if canonical_img_id is None:
                logger.warning(
                    "Shard %s: annotation references unknown image_id=%s; "
                    "skipping.",
                    shard_path,
                    local_img_id,
                )
                continue
            new_ann = dict(ann)
            new_ann["image_id"] = canonical_img_id
            new_ann["id"] = next_ann_id
            merged["annotations"].append(new_ann)
            next_ann_id += 1

    _atomic_dump_json(out_path, merged)
    logger.info(
        "Wrote %s (%d images, %d annotations from %d shards).",
        out_path,
        len(merged["images"]),
        len(merged["annotations"]),
        len(shard_names),
    )
    return merged


def split_annotations(
    in_path: str, output_dir: Optional[str] = None
) -> int:
    """Split a monolithic ``annotations.json`` back into per-pair shards.

    For every ``images[]`` entry in ``in_path``, a shard is written at
    ``<output_dir>/pairs/<stem>__<np_slug>.json`` whose ``images`` list
    contains just that entry (renumbered to local id 1) and whose
    ``annotations`` list contains exactly the annotations whose
    ``image_id`` matched the original entry (renumbered to local
    ids 1..N with ``image_id = 1``).

    Useful for migrating legacy labeler output (pre-shard) into the new
    layout. Returns the number of shards written.
    """
    if output_dir is None:
        output_dir = os.path.dirname(os.path.abspath(in_path))
    output_dir = os.path.abspath(output_dir)
    pairs_dir = os.path.join(output_dir, "pairs")
    os.makedirs(pairs_dir, exist_ok=True)

    with open(in_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    images: List[Dict[str, Any]] = data.get("images") or []
    annotations: List[Dict[str, Any]] = data.get("annotations") or []

    # Bucket annotations by their (canonical) image_id for O(1) lookup.
    by_image_id: Dict[int, List[Dict[str, Any]]] = {}
    for ann in annotations:
        by_image_id.setdefault(int(ann.get("image_id", -1)), []).append(ann)

    written = 0
    for img in images:
        canonical_id = int(img.get("id", -1))
        file_name = img.get("file_name") or ""
        np_text = img.get("text_input") or ""
        if not file_name:
            logger.warning(
                "Skipping images[] entry id=%s with empty file_name.",
                canonical_id,
            )
            continue

        local_img = dict(img)
        local_img["id"] = 1

        local_anns: List[Dict[str, Any]] = []
        for i, ann in enumerate(by_image_id.get(canonical_id, []), start=1):
            new_ann = dict(ann)
            new_ann["id"] = i
            new_ann["image_id"] = 1
            local_anns.append(new_ann)

        shard = {
            "info": dict(data.get("info") or _DEFAULT_INFO),
            "categories": list(data.get("categories") or _DEFAULT_CATEGORIES),
            "images": [local_img],
            "annotations": local_anns,
        }

        stem = os.path.splitext(os.path.basename(file_name))[0]
        slug = _slugify(np_text or "np")
        shard_path = os.path.join(pairs_dir, f"{stem}__{slug}.json")
        _atomic_dump_json(shard_path, shard)
        written += 1

    logger.info(
        "Split %s into %d shards under %s.", in_path, written, pairs_dir
    )
    return written


# ---------------------------------------------------------------------- #
# CLI
# ---------------------------------------------------------------------- #
def _parse_args(argv: List[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        prog="python -m sam3.labeler.compact",
        description=(
            "Merge per-pair labeler shards into a canonical "
            "annotations.json, or split an existing annotations.json "
            "back into shards (--split)."
        ),
    )
    p.add_argument(
        "output_dir",
        help=(
            "Labeler output directory (the one that contains pairs/, "
            "vis/, prompts/, etc.)."
        ),
    )
    p.add_argument(
        "--split",
        action="store_true",
        help=(
            "Read an existing monolithic annotations.json and write "
            "one shard per (image, NP) pair into pairs/."
        ),
    )
    p.add_argument(
        "--in",
        dest="in_path",
        default=None,
        help=(
            "[--split only] Path to the legacy annotations.json. "
            "Defaults to <output_dir>/annotations.json."
        ),
    )
    p.add_argument(
        "--out",
        dest="out_path",
        default=None,
        help=(
            "[default mode] Path to write the merged annotations.json. "
            "Defaults to <output_dir>/annotations.json."
        ),
    )
    p.add_argument(
        "-v",
        "--verbose",
        action="store_true",
        help="Enable INFO-level logging.",
    )
    return p.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = _parse_args(argv if argv is not None else sys.argv[1:])
    logging.basicConfig(
        level=logging.INFO if args.verbose else logging.WARNING,
        format="%(levelname)s %(name)s: %(message)s",
    )

    if args.split:
        in_path = args.in_path or os.path.join(
            args.output_dir, "annotations.json"
        )
        if not os.path.exists(in_path):
            print(
                f"error: --split source not found: {in_path}", file=sys.stderr
            )
            return 2
        n = split_annotations(in_path, args.output_dir)
        print(f"Wrote {n} shard(s) under {args.output_dir}/pairs/.")
        return 0

    merged = compact_pairs(args.output_dir, args.out_path)
    out_path = args.out_path or os.path.join(
        args.output_dir, "annotations.json"
    )
    print(
        f"Wrote {out_path}: {len(merged['images'])} image-NP pair(s), "
        f"{len(merged['annotations'])} annotation(s)."
    )
    return 0


if __name__ == "__main__":  # pragma: no cover - CLI entry point
    raise SystemExit(main())
