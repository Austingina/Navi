# SAM 3 Browser Annotation Tool

A Gradio app for labelling images with SAM 3. It exposes both of the
model's image-inference paths against a single cached image embedding,
plus an edit mode for fixing up detections that already exist on disk:

| Mode                            | Backend call                                                            | Prompts                                                       | Save semantics                                                                |
|---------------------------------|-------------------------------------------------------------------------|---------------------------------------------------------------|-------------------------------------------------------------------------------|
| **Find all (PCS)**              | `Sam3Processor.set_text_prompt` + `Sam3Processor.add_geometric_prompt`  | A noun phrase, plus optional positive / negative box exemplars | **Replaces** the `(image, NP)` shard wholesale (PCS re-finds every instance). |
| **Click and segment (PVS)**     | `Sam3Image.predict_inst`                                                | Per-object positive / negative point clicks + optional box     | **Appends** to the existing `(image, NP)` shard, honouring any pending deletes from edit mode. |
| **Edit existing detections**    | None (pure local edit of saved shards)                                  | Point / box clicks on the image canvas                         | Stages deletes in memory; **Save edits to disk** rewrites shards via overrides. |

Every **Save** writes (or rewrites) one self-contained JSON shard at
`<output_dir>/pairs/<image_stem>__<np_slug>.json`. PCS rewrites it
from scratch; PVS merges new detections into whatever is already there;
edit mode drops selected annotations. The canonical
`annotations.json` that SAM 3 eval / training expects is produced from
those shards on demand by `python -m sam3.labeler.compact <output_dir>`
(see [Compacting shards into annotations.json](#compacting-shards-into-annotationsjson)).
To score model predictions against compacted labels, use
`python -m sam3.labeler.evaluate` (see
[Comparing labeler output against model predictions](#comparing-labeler-output-against-model-predictions)).
The shard and the merged file share the COCO-NP-pair schema documented in
[`sam3/scripts/eval/gold/README.md`](../../scripts/eval/gold/README.md)
and consumed by the loaders in
[`sam3.train.data.sam3_image_dataset`](../train/data/sam3_image_dataset.py).

## Install

From the repo root:

```bash
# Inside your sam3 conda / venv environment
pip install -e ".[labeler]"
```

This pulls `gradio>=5,<6`, `pycocotools`, `opencv-python`, and `Pillow` on top of the base `sam3` install. Gradio 5.x is required because Gradio 4.x clashes with modern Starlette (template-response signature change) and also tries to import the removed `HfFolder` symbol from `huggingface_hub` 1.x. Gradio 5 plays cleanly with current `starlette`, `huggingface_hub>=1.x`, and `transformers`, so the TRT export pipeline in [`sam3/export_hf`](../../export_hf) keeps working in the same env.

If you previously force-installed older versions while iterating, do a one-shot upgrade:

```bash
pip install -e ".[labeler]" --upgrade \
    "gradio>=5,<6" gradio_client "huggingface_hub>=1.0"
```

The `sam3` package weights are downloaded from HuggingFace (`facebook/sam3`) the first time you run inference, unless you point `--checkpoint_path` at a local file.

## Run

```bash
python -m sam3.labeler.app \
    --output_dir /tmp/sam3-labels \
    --device cuda:0
```

Optional flags:

```
--bpe_path PATH            Override the BPE vocab (defaults to the one bundled in sam3/assets)
--checkpoint_path PATH     Override the SAM 3 checkpoint location
--share                    Launch a public Gradio URL
--server_name 0.0.0.0      Bind interface. Default is 0.0.0.0 (LAN / SSH host reachable).
                           Use 127.0.0.1 for loopback-only.
--server_port 7860
--show_api                 Enable Gradio's /info schema endpoint. Off by default
                           because some gradio_client builds crash with
                           "argument of type 'bool' is not iterable" on it.
--preload                  Load the SAM 3 model at startup instead of on first Detect
--max_cached_states 8      LRU cap on cached per-image embeddings (controls VRAM)
```

Open the printed URL in a browser. If you are SSH'd into a remote
host, point your browser at `http://<host-ip>:7860` (the server binds
to all interfaces by default).

## Using the UI

### Common controls

1. Pick a **Source**:
   - *Single upload* — drop or pick an image file in the **Upload an
     image** widget. (Webcam and clipboard sources are intentionally
     disabled on the click target so it stays a pure prompt canvas;
     all uploads go through that widget.)
   - *Directory* — type a folder path and click **Load directory**, then
     step through with **Prev / Next**. The image counter shows your
     position.
2. Pick a **Mode** — *Find all (PCS)*, *Click and segment (PVS)*, or
   *Edit existing detections* (review and delete shards that were
   previously saved by this app or by
   [`trt_infer.py --save_for_labeler`](../../export_hf/trt_infer.py);
   see [Edit existing detections](#edit-existing-detections) below).
3. (PCS only) Type a **Noun phrase** describing what you want SAM 3 to
   find. In PVS the NP is recorded as the label of every saved mask but
   is not sent to the model. In edit mode the NP field is hidden -
   shards already carry their NP on disk.
4. Set the **Confidence threshold** (PCS) or the **multimask output**
   toggle (PVS). Higher confidence in PCS means fewer but more confident
   detections; in PVS, multimask asks SAM 3 to propose three candidate
   masks and keeps the best one by score.
5. Use the **Tool** radio to pick *Point* or *Box*, and the **Polarity**
   radio to tag the next prompt as *Positive* or *Negative*. Each
   left-click on the input image drops one prompt and the image
   redraws with all current prompts overlayed on it (no separate
   overlay panel). Detections that already exist on disk for the
   current image are drawn as a faint cyan layer behind the prompts
   in PCS and PVS modes, so you can place new clicks with full
   awareness of what is already detected; this layer is purely visual
   and is never sent to the model.
   - With *Tool = Point*: a point at the click location.
   - With *Tool = Box*: the first click places the top-left corner (a
     yellow cross-hair marks it), the second click places the
     bottom-right corner and commits the box. The state resets after
     every committed box, so the next click begins a new one.

### What each click actually does

The six useful Mode × Tool combinations are:

| Mode | Tool | Click behaviour |
|------|------|-----------------|
| Find all (PCS) | Box | Place positive (green) or negative (red) box exemplars to disambiguate the noun phrase. Two clicks per box. |
| Find all (PCS) | Point | SAM 3's text path does not accept points, so the click is auto-converted to a small ±12 px box. Use this only as a quick exemplar; prefer the Box tool. |
| Click and segment (PVS) | Point | Positive points go inside the object, negative points stay off it. Click **Add object (PVS)** to commit the slot and start the next object. |
| Click and segment (PVS) | Box | Optional single bounding box around the current PVS object slot, drawn with two clicks. Combines with the slot's points. |
| Edit existing detections | Point | Toggle selection on the smallest existing detection under the click. Picker is forgiving: a mask hit wins, otherwise the smallest bbox containing the click wins, otherwise any bbox within ~16 px of the click wins (so label-tag and outline clicks still count). |
| Edit existing detections | Box | Two-click rectangle; every loaded detection whose bbox centre falls inside is added to the selection. Use it for batch cleanup. |

### Find all (PCS)

- Type the noun phrase.
- Optionally place positive (green) or negative (semi-transparent red)
  box exemplars around concrete instances to disambiguate the concept.
  Set **Tool = Box** and click twice per box.
- If you accidentally click while **Tool = Point** in PCS, the click is
  converted to a tiny `±12 px` box around it (since
  `Sam3Processor.add_geometric_prompt` does not accept point prompts).
- Click **Detect** to get every matching instance.
- Click **Save** to write one self-contained shard at
  `pairs/<image_stem>__<np_slug>.json` (one image-NP pair plus one
  annotation per predicted mask, with local ids starting at 1). The
  visualisation is written to `vis/<image_stem>__<np_slug>.jpg` and the
  raw prompts to `prompts/<image_stem>__<np_slug>.json`. Re-saving the
  same image-NP pair **overwrites** that shard rather than accumulating
  duplicates.
- Click **Mark as negative** to record that the NP is *absent* from the
  current image (a shard with an empty `annotations[]` list).

### Click and segment (PVS)

- Set **Tool = Point** and click positive (green) or negative (red)
  points on the object you want. Each point shows up as a coloured dot
  with a thin white outline directly on the input image.
- (Optional) Switch to **Tool = Box** and click the top-left then
  bottom-right corners to add a single box for the current object.
- Click **Add object** to commit the current slot and start a new one.
  Each committed slot gets a colour-coded label (`obj 1`, `obj 2`, ...).
- Repeat for each instance you want, then **Detect**. SAM 3's
  `predict_inst` runs once per committed slot (plus the currently
  uncommitted one if it has any prompts), reusing the cached image
  embedding.
- **Reset prompts** clears only the current uncommitted slot;
  **Reset all (PVS)** wipes every slot.
- **Save** writes one shard under `pairs/<stem>__<np_slug>.json` with
  one annotation per committed slot, all sharing the NP as their label.
  **Mark as negative** still works but is typically only useful in PCS.

### Edit existing detections

Use this mode to clean up shards that already live on disk, whether
they were saved by the **PCS / PVS** flows above or produced by
[`sam3/export_hf/trt_infer.py --save_for_labeler`](../../export_hf/trt_infer.py),
which writes the same `pairs/<stem>__<np_slug>.json` layout (with
RLE masks) so detection results are directly editable here.

**Setup**

1. Set the **Output directory** (under *Save options*) to the folder
   that holds `pairs/` — for example
   `/path/to/inference_outputs/shards`, **not** the `pairs/`
   sub-folder. The store appends `pairs/<stem>__<np>.json` itself.
2. Load an image (single upload or *Directory* + Load directory).
   The app automatically scans
   `<output_dir>/pairs/<stem>__*.json` for that image and renders all
   matching detections as cyan outlines on the input canvas.
3. Switch the **Mode** radio to *Edit existing detections*. If you
   change the output directory after loading the image, click
   **Reload from disk** to refresh.

**Selecting detections**

- **Point tool** — left-click toggles selection of one detection.
  Selected detections turn red with a thick outline and a `*` prefix
  on their label.
- **Box tool** — two left-clicks draw a rectangle; every detection
  whose bbox centre is inside is added to the selection. Use it to
  batch-clean a region.
- **Clear selection** un-highlights without changing anything.

**Deleting and committing (deferred-save workflow)**

Deletions are staged in memory so you can preview the result before
touching disk:

1. **Delete selected** — moves currently-selected detections into an
   in-memory *pending delete* queue. They immediately **disappear
   from the canvas** so you see exactly what the post-save image will
   look like. The shards on disk are still untouched.
2. **Save edits to disk** (primary button) — rewrites the affected
   `pairs/<stem>__<np>.json` shards via
   `AnnotationStore.write_pair_overrides`, dropping only the queued
   ann ids. Local `id` fields are re-numbered to stay contiguous
   inside each shard. The in-memory state is then reloaded from disk
   so the canvas matches what compact will read next.
3. **Discard edits** — empties the pending queue. The hidden
   detections reappear; disk is unchanged.
4. **Reload from disk** — throws away both the selection and the
   pending queue, then re-reads the shards. Use this if you edited
   shards externally and want to resync.

The status bar at the bottom reads e.g.
`frame000009.jpg (1280x1280) | Edit mode: 12 loaded across 4 NP(s); 1 selected; 2 pending delete (unsaved).`
If the loaded count is `0`, the output directory is wrong (or the
shard layout is something other than `pairs/<stem>__*.json`).

After saving edits, run `python -m sam3.labeler.compact <output_dir>`
to refresh the canonical `annotations.json` for downstream eval /
training the same way you would after a normal PCS / PVS save.

#### Combining edit mode with PVS to fix missed objects

Edit-mode pending deletes and PVS Save are wired together: a single
**Save** click in PVS mode can both drop edit-mode-queued detections
and append new ones, atomically, against the same shard. Use this to
clean up an inference run and add a missed object in one pass:

1. Point the **Output directory** at the
   `trt_infer.py --save_for_labeler` output (the folder that contains
   `pairs/`).
2. Switch to **Edit existing detections**. Click false positives,
   then **Delete selected** to queue them. They disappear from the
   canvas but are not yet written.
3. Switch to **Click and segment (PVS)**. The remaining detections
   stay visible as a faint cyan overlay so you can see what is
   already covered. Click positive / negative points (and optionally
   a box) on the missed object, **Add object** if you want more
   slots, then **Detect**.
4. Click **Save**. The shard is rewritten with `(originals - queued
   deletes + new PVS masks)`; local annotation ids are renumbered
   contiguously, and the original `images[]` row (`text_input`,
   exhaustiveness flags, etc.) is preserved verbatim.

The status bar after the merged save reads e.g.
`Appended 1 PVS detection(s) and dropped 2 pending delete(s) for NP
'door' (5 total now). Wrote shard frame000037__door.json and
frame000037__door.jpg.` After the write, both the canvas and the
`vis/<stem>__<np>.jpg` thumbnail reflect the merged result list, so
you can compare it against the original `vis_<stem>.jpg` produced by
the inference run.

PCS Save still **replaces** the shard wholesale (PCS finds every
matching instance from text, so a fresh enumeration is the right
semantic). Both PCS Save and Mark-as-negative drop any pending
deletes scoped to that NP after the write because their stale ann
ids no longer exist on disk.

## On-disk layout after one Save

```
/tmp/sam3-labels/
├── pairs/
│   └── img_001__a-red-mug.json    # one self-contained shard, written by Save
├── vis/
│   └── img_001__a-red-mug.jpg
└── prompts/
    └── img_001__a-red-mug.json
```

Note that **`annotations.json` is intentionally absent from this
listing**: the labeler never writes it directly. It is produced from
the shards on demand by `python -m sam3.labeler.compact <output_dir>`
(see below).

A single shard at `pairs/img_001__a-red-mug.json` looks like (one
image-NP pair, one annotation per mask, bbox normalized xywh,
segmentation in COCO RLE; `images[].id` and `annotations[].image_id`
are always `1` within a shard):

```json
{
  "info": {"version": "0.1", "tool": "sam3-labeler"},
  "categories": [{"id": 1, "name": "object"}],
  "images": [{
    "id": 1,
    "file_name": "img_001.jpg",
    "width": 1920,
    "height": 1080,
    "text_input": "the red mug",
    "queried_category": "1",
    "is_instance_exhaustive": 1,
    "is_pixel_exhaustive": 1,
    "mode": "pcs",
    "input_box": [[0.40, 0.50, 0.05, 0.05]],
    "input_box_label": [1]
  }],
  "annotations": [{
    "id": 1,
    "image_id": 1,
    "category_id": 1,
    "iscrowd": 0,
    "bbox": [0.41, 0.36, 0.08, 0.12],
    "area": 0.0096,
    "segmentation": {"counts": "<RLE>", "size": [1080, 1920]},
    "score": 0.92,
    "source": "sam3-pcs"
  }]
}
```

The `bbox` field is normalized `xywh` (each in `[0, 1]`) so the
file is image-resolution-agnostic. The training loaders in
[`sam3/train/data/sam3_image_dataset.py`](../train/data/sam3_image_dataset.py)
multiply by `(W, H)` on the way in.

## Compacting shards into `annotations.json`

When you are ready to feed labels into SAM 3 eval or training, merge
all shards in an output directory into the canonical file:

```bash
# Builds <output_dir>/annotations.json from <output_dir>/pairs/*.json.
python -m sam3.labeler.compact /tmp/sam3-labels
```

The merge sorts shards by filename, renumbers `images[].id`
monotonically starting at 1, rewrites every `annotations[].image_id`
accordingly, and reassigns `annotations[].id` across the whole
directory. Re-running on an unchanged `pairs/` produces a byte-identical
`annotations.json`. A malformed or unreadable shard is logged and
skipped rather than aborting the merge.

To delete a mislabelled pair, just remove the shard (and optionally its
sidecars) and re-run `compact`:

```bash
rm pairs/img_001__a-red-mug.json vis/img_001__a-red-mug.jpg prompts/img_001__a-red-mug.json
python -m sam3.labeler.compact /tmp/sam3-labels
```

If you have a legacy `annotations.json` produced by an older labeler
revision (or by hand-editing) and want to drop back into the shard
workflow, run the migration helper. It splits the monolithic file into
one shard per `(file_name, text_input)` pair:

```bash
# Reads <output_dir>/annotations.json and emits one shard per pair
# into <output_dir>/pairs/.
python -m sam3.labeler.compact /tmp/sam3-labels --split

# Or point at an arbitrary source file:
python -m sam3.labeler.compact /tmp/sam3-labels --split --in legacy.json
```

`--out PATH` lets you redirect the merged output to a custom location;
omit it and the canonical `<output_dir>/annotations.json` is written
atomically.

## Smoke test

After install, you can verify the modules import and that the shard
writer + compact step produce well-formed JSON without launching the
UI:

```bash
python - <<'PY'
import warnings, tempfile, json, os
import numpy as np
warnings.filterwarnings("ignore")
from sam3.labeler.annotations import AnnotationStore
from sam3.labeler.compact import compact_pairs

tmpd = tempfile.mkdtemp(prefix="sam3-labeler-smoke-")
store = AnnotationStore(tmpd)
mask = np.zeros((100, 200), bool); mask[20:80, 30:170] = True
store.save_pair(
    file_name="example.jpg", width=200, height=100,
    noun_phrase="a red mug",
    results=[{"mask": mask, "box_xyxy_pix": (30, 20, 170, 80), "score": 0.9, "source": "sam3-pcs"}],
    pcs_pos_boxes_xyxy_pix=[(25, 15, 175, 85)],
    mode="pcs",
)
shard_path = os.path.join(tmpd, "pairs", "example__a-red-mug.json")
shard = json.load(open(shard_path))
print("shard images:", len(shard["images"]), "annotations:", len(shard["annotations"]))
print("counts is str:", isinstance(shard["annotations"][0]["segmentation"]["counts"], str))

merged = compact_pairs(tmpd)
print("merged images:", len(merged["images"]), "annotations:", len(merged["annotations"]))
print("layout:", sorted(os.listdir(tmpd)))
PY
```

Expected output (the temp directory varies):

```
shard images: 1 annotations: 1
counts is str: True
merged images: 1 annotations: 1
layout: ['annotations.json', 'pairs', 'prompts', 'vis']
```

## Comparing labeler output against model predictions

The three offline inference scripts emit an `predictions.json` in the
same labeler schema (bbox-only, no RLE segmentation). The labeler
itself writes per-pair shards, so build its canonical `annotations.json`
once with `compact` before comparing against a fresh model run:

```bash
# Label, then compact into the canonical annotations.json.
python -m sam3.labeler.app --output_dir /tmp/labels ...
python -m sam3.labeler.compact /tmp/labels

# Re-run any of the three inference scripts on the same images;
# each writes its own annotations.json to its --output_dir directly
# (inference scripts do NOT use the shard layout).
python sam3/export_hf/trt_infer.py     --input_dir /path/to/images --output_dir /tmp/preds-trt
python sam3/scripts/hf_infer.py        --input_dir /path/to/images --output_dir /tmp/preds-hf
python sam3/scripts/parallel_infer.py  --input_dir /path/to/images --output_dir /tmp/preds-parallel
```

What is and is not consistent across the files:

- **Schema**: identical to the labeler. Both files have `info`,
  `images[]`, `annotations[]`, `categories`, with the same field
  names and the same normalized `xywh` bbox in `[0, 1]`.
- **Provenance**: `info.tool` is set to `sam3-trt-infer`,
  `sam3-hf-infer`, or `sam3-parallel-infer`, and every
  `annotations[].source` is `trt-pcs` / `hf-pcs` / `parallel-pcs`. The
  labeler uses `sam3-pcs` / `sam3-pvs`, so a diff can tell at a glance
  which file came from where.
- **Image IDs are file-local**: each run numbers `images[].id` from 1,
  so when joining a labeler file against an inference file you must
  match by `(file_name, text_input)` on the `images[]` rows, not by
  `id`. The training loaders in
  [`sam3/train/data/sam3_image_dataset.py`](../train/data/sam3_image_dataset.py)
  treat each `(file_name, text_input)` as a unique pair, which is the
  same join key used by `sam3.labeler.evaluate`.
- **Negative pairs**: each inference run records one `images[]` row
  per queried NP even when zero instances were detected, matching the
  labeler's `Mark as negative` convention. This lets downstream
  evaluators (`pycocotools` COCOeval or
  [`sam3/scripts/eval/standalone_cgf1.py`](../../scripts/eval/standalone_cgf1.py))
  compute false negatives properly.
- **No masks**: the inference files intentionally omit
  `annotations[].segmentation` to stay fast and small. Bbox IoU is
  enough for `cgF1` / COCOeval detection-style metrics; if you later
  need mask IoU, extend
  `InferenceAnnotationBuilder.add_pair` with RLE encoding.
- **Multi-GPU sharding (`parallel_infer.py`)**: each shard writes its
  own `annotations.json` into its `--output_dir`; if shards share an
  output directory they will overwrite each other. Either use per-shard `--output_dir`s or
  rename the file post-run before merging.

### Macro precision with `sam3.labeler.evaluate`

[`sam3/labeler/evaluate.py`](evaluate.py) compares a ground-truth
`annotations.json` (from the labeler, after `compact`) against a model
`predictions.json` and prints **macro precision**
plus per-NP precision and recall. It uses bbox IoU only (no mask IoU),
COCO-style greedy matching per `(file_name, text_input)` pair, and
predictions sorted by `score` descending.

```bash
python -m sam3.labeler.evaluate \
    --gt   /tmp/labels/annotations.json \
    --pred /tmp/preds-trt/annotations.json \
    --iou 0.5

# Several IoU thresholds in one run
python -m sam3.labeler.evaluate ... --iou 0.5 --iou 0.75

# Drop low-confidence predictions before matching
python -m sam3.labeler.evaluate ... --score 0.30

# Machine-readable JSON instead of the ASCII table
python -m sam3.labeler.evaluate ... --json > metrics.json
```

**Definitions** (for each IoU threshold):

- **TP**: a prediction matched to an unmatched GT box with IoU ≥
  threshold (each GT box matches at most one prediction).
- **FP**: a prediction with no matching GT box above the threshold.
- **Per-NP precision** = `sum(TP) / sum(TP + FP)` over all images where
  that NP was evaluated.
- **Per-NP recall** = `sum(TP) / sum(#GT boxes)` over those same images
  (FNs are unmatched GT boxes; not shown as a separate column but
  derivable from TP and `#GT`).
- **Macro precision** = unweighted mean of per-NP precision over NPs
  that have at least one prediction (`TP + FP > 0`). NPs with zero
  predictions are reported as `n/a` and excluded from the macro mean
  (precision is undefined when the model made no calls for that class).

This is **macro** (each noun phrase counts equally), not **micro**
(instance-weighted `sum(TP) / sum(TP + FP)` across all boxes).

### Which `(file, NP)` pairs are evaluated: `--include_pred_only_pairs`

Both JSON files are sets of **image-NP pairs**: each `images[]` row means
“the model was queried for NP *X* on image *Y*”. Evaluation always joins
on `(file_name, text_input)`, never on `images[].id`.

| Pair kind | In ground truth? | In predictions? | Default (`--include_pred_only_pairs` off) | With flag on |
|-----------|------------------|-----------------|-------------------------------------------|--------------|
| **Labeled pair** | yes | yes or no | Evaluated: preds matched to GT boxes | Same |
| **Negative GT pair** (Mark as negative) | yes, `annotations[]` empty | any | Evaluated: every prediction is an FP | Same |
| **Pred-only pair** | **no** | yes | **Ignored**: predictions on this pair are dropped and do not affect any NP’s TP/FP | **Evaluated**: GT list is empty, so every prediction is an FP |

**Pred-only** means the inference run queried an `(image, NP)` that you
never saved a label for. Example: you labeled `frame002326.jpg` for
`person`, `road`, and `sidewalk`, but the TRT script also queried
`vehicle` on that frame. With the default (strict) mode, those vehicle
boxes are invisible to the metric. With `--include_pred_only_pairs`,
they become false positives for the `vehicle` NP because there is no GT
to match them against—even if vehicles are actually present and the
model is correct.

**When to use which mode**

- **Default (off)** — “Among the queries I labeled, how accurate are the
  predictions?” This is the usual choice when your ground truth defines
  the evaluation universe (same spirit as restricting COCOeval to
  labeled categories).
- **`--include_pred_only_pairs` (on)** — “Among every query in the
  prediction file, how accurate are the predictions?” This penalizes the
  model for detections on unlabeled queries. Use it only if you intend
  open-world scoring or you have labeled every query the inference
  script made.

If inference queried more `(file, NP)` pairs than you labeled, the
strict and open-world macro precision numbers will differ only when
pred-only pairs contain non-zero predictions. To align them, either
label the missing pairs (use **Mark as negative** when the NP is absent)
or restrict inference to the same query list as your labels.

**Pred-only vs negative GT** — do not confuse them:

- A **negative GT pair** is in your `annotations.json` with zero
  `annotations[]` rows: you explicitly said “this NP is not on this
  image.” Predictions there are always FPs in both modes.
- A **pred-only pair** is absent from ground truth entirely: the
  evaluator has no statement about that query unless you pass
  `--include_pred_only_pairs`.

## Notes and edge cases

- The SAM 3 model is loaded lazily on the first **Detect**, so app
  startup is cheap. Pass `--preload` to load it eagerly.
- `Sam3Annotator` caches per-image embeddings in an LRU keyed by file
  path (`--max_cached_states`, default 8). Toggling modes or NPs on the
  same image is therefore very fast; only `Sam3Processor.set_image`
  costs more than the prompt forward pass.
- Switching modes preserves the prompts of the other mode, so you can
  A/B test PCS vs PVS on the same image without re-clicking.
- Re-running the app against an existing `output_dir` is safe: shards
  for new `(image, NP)` pairs land alongside existing ones, and a
  re-save of an existing pair overwrites only its own shard. The
  canonical `annotations.json` is regenerated from scratch (with fresh
  monotonic IDs) each time you run `compact`, so deleted shards
  disappear cleanly without orphan IDs.
- The UI uses only native Gradio 5 components, so it stays compatible
  with current `huggingface_hub` and `transformers`. The trade-off is
  that boxes are placed with **two clicks** (top-left + bottom-right)
  rather than a single click-drag.
- The input image and the prompt overlay share a single widget. Every
  click pushes a freshly rendered overlay back into the same image, so
  click coordinates always correspond to original-image pixels.
- `Mark as negative` writes an `images[]` entry with no matching
  `annotations[]` rows. This is exactly how SAM 3's evaluation data
  represents "this NP does not appear in this image".
- The edit-mode *pending delete* queue lives in the Gradio session
  state only. Closing the browser tab, switching to a different image,
  or hitting **Reload from disk** all discard the queue without
  touching the shards. Always click **Save edits to disk** before
  navigating away if you want the deletion to stick.
