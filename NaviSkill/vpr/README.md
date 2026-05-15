# VPR Pipeline V1

Visual Place Recognition pipeline with selectable global descriptors (MegaLoc or SALAD-indoor), FAISS retrieval, and optional re-ranking stages (geometric and/or color).

## Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                        VPR Pipeline V1                           │
│                                                                  │
│  ┌──────────────────────────────────────────────────────────┐    │
│  │ Stage 1: Global Descriptor Extraction                     │    │
│  │                                                           │    │
│  │  Image ──► DINOv2 Backbone ──► Aggregator ──► 8448-d vec  │    │
│  │                                                           │    │
│  │  Models:                                                  │    │
│  │  • megaloc      — outdoor/geometry, color-invariant       │    │
│  │  • salad-indoor — indoor fine-tuned, appearance-sensitive │    │
│  └──────────────────────────────────────────────────────────┘    │
│                              │                                   │
│                              ▼                                   │
│  ┌──────────────────────────────────────────────────────────┐    │
│  │ Stage 2: kNN Retrieval (FAISS)                            │    │
│  │  Query descriptor ──► L2 distance to all DB ──► top-k     │    │
│  └──────────────────────────────────────────────────────────┘    │
│                              │                                   │
│                    ┌─────────┴─────────┐                         │
│                    ▼                   ▼                          │
│  ┌──────────────────────┐  ┌──────────────────────────────┐     │
│  │ Stage 3: Geometric   │  │ Stage 4: Color Re-scoring    │     │
│  │ Re-ranking           │  │ [--color_rerank]             │     │
│  │ [--rerank]           │  │                              │     │
│  │                      │  │  HSV histogram similarity    │     │
│  │ SuperPoint+LightGlue │  │  combined with base scores   │     │
│  │ inlier counting      │  │  score = base*(1+α*color)    │     │
│  └──────────────────────┘  └──────────────────────────────┘     │
│                    │                   │                          │
│                    └─────────┬─────────┘                         │
│                              ▼                                   │
│                     Final Predictions                            │
│              (ranked candidates + confidence)                    │
└──────────────────────────────────────────────────────────────────┘
```

### Recommended configurations

| Scenario | Method | Re-ranking | Expected accuracy |
|----------|--------|------------|-------------------|
| **Cross-floor recognition** | `salad-indoor` | `--color_rerank --color_weight=5.0` | ~99% |
| **Same-floor localization** | `megaloc` | `--rerank` | ~90-96% R@1 |
| **Fast retrieval (no rerank)** | `salad-indoor` | *(none)* | ~89% cross-floor |

## File Structure

```
NaviSkill/vpr/             # VPR module of NaviSkill (was vpr/vpr-pipeline-v1/)
├── README.md              # This file
├── pyproject.toml         # Package metadata + dependencies (uv / pip)
├── requirements.txt       # Legacy dependency list
├── uv.lock                # uv lockfile
├── install_jetson.sh      # Jetson-specific torch/torchvision install helper
│
├── vpr_preprocess.py      # ★ Live pipeline: build per-submap descriptor DB
├── vpr_detect.py          # ★ Live pipeline: query detector (--serve mode)
├── model_loader.py        # MegaLoc + SALAD-indoor model loading
│
├── utils/                 # ★ Shared library code (no CLI / no main)
│   ├── __init__.py
│   ├── utils_common.py    #   Platform-agnostic helpers (DistanceSampler,
│   │                      #   JPEG save/undistort, pose loaders, summaries)
│   ├── utils_s20.py       #   SHARE SLAM S20 (ROS1 bag) extraction
│   ├── utils_r1pro.py     #   R1Pro (ROS2 bag) extraction + worker init
│   ├── undistort.py       #   Fisheye undistortion (PolyFisheye)
│   ├── undistort2.py      #   Pinhole undistortion (R1Pro)
│   ├── rerank.py          #   SuperPoint + LightGlue re-ranking (eval-only)
│   └── color_rerank.py    #   HSV color histogram re-scoring (eval-only)
│
└── eval/                  # ★ Offline evaluation / algorithm-feasibility code
    ├── build_database_3d.py  # Legacy CLI: build DB + queries split
    ├── dataset_3d.py         # Eval Dataset (DB + queries pair, recall@K)
    ├── pipeline_3d.py        # Eval pipeline: FAISS + optional re-ranking
    └── visualizations.py     # Prediction visualization (used by pipeline_3d)
```

## Setup

```bash
# Python 3.10+ required — use uv for dependency management
cd NaviSkill/vpr
uv sync                    # Creates .venv and installs all deps

# For ROS2 bag support only: requires ROS2 Humble system install
# (rosbag2_py, rclpy, sensor_msgs, nav_msgs are system packages — use native Python)
```

## Live Pipeline

Two top-level scripts drive the **production** pipeline:

* `vpr_preprocess.py` — build a per-submap descriptor database from one ROS bag
  (one bag per submap), invoked during map authoring (also wired into MapEditor's
  "Build VPR database" dialog).
* `vpr_detect.py` — query the database(s); supports a one-shot CLI mode and a
  long-running `--serve` mode that talks JSON over stdin/stdout (used by
  `NaviSkill/vpr_bootstrap.py` from `server.py`).

### Build a database — `vpr_preprocess.py`

Two input sources, selected via `--source`:

* `--source=ros1` (default) — SHARE SLAM **S20** ROS1 `.bag` + `ImgPose.txt`.
  Optional polyfisheye → perspective undistortion via `--undistort_fisheye`.
* `--source=ros2_r1pro` — **R1Pro** ROS2 `.db3` with `/Odometry` +
  `CompressedImage`. Pinhole-undistorted and center-cropped automatically to
  match the geometry baked into `vpr_bootstrap`.

```bash
# S20 — ROS1 bag, fisheye undistortion → 1024x768 perspective
uv run python vpr_preprocess.py \
    --source=ros1 \
    --bag_path=/data/maps/408/floor2/floor2.bag \
    --pose_file=/data/maps/408/floor2/ImgPose.txt \
    --image_topic=/camera_agent/img_left/compressed \
    --undistort_fisheye \
    --db_distance=1.0 \
    --image_cache_dir=~/.vpr/408/floor2 \
    --output_dir=/home/nvidia/Navi/maps/408/vpr/floor2 \
    --method=salad-indoor --device=cuda

# R1Pro — ROS2 bag, automatic pinhole undistortion + center crop to 1024x768
uv run python vpr_preprocess.py \
    --source=ros2_r1pro \
    --bag_path=/data/rosbag_20260514_093133 \
    --pose_topic=/Odometry \
    --image_topic=/hdas/camera_head/left_raw/image_raw_color/compressed \
    --db_distance=1.0 \
    --crop_width=1024 --crop_height=768 \
    --image_cache_dir=~/.vpr/408/floor2 \
    --output_dir=/home/nvidia/Navi/maps/408/vpr/floor2 \
    --method=salad-indoor --device=cuda
```

Output (`--output_dir`) is exactly what `vpr_detect.py` reads back:

```
<output_dir>/
├── descriptors.npy            # (N, D) float32 L2-normalized global descriptors
├── image_paths.json           # paths to the cached query JPEGs (under image_cache_dir)
├── poses.npy                  # (N, 3) tx ty tz per image
└── database_build_info.json   # method, source, undistort/crop config, etc.
```

Decoded JPEGs themselves live under `--image_cache_dir` (default
`~/.vpr/<project>/<submap>`), so `--output_dir` stays small enough to keep
under the map repo.

### Query a database — `vpr_detect.py`

One-shot CLI (sanity-check a freshly built database):

```bash
uv run python vpr_detect.py \
    --image /tmp/query.jpg \
    --databases /home/nvidia/Navi/maps/408/vpr/floor1 \
                /home/nvidia/Navi/maps/408/vpr/floor2 \
                /home/nvidia/Navi/maps/408/vpr/floor3 \
    --top_k 3 \
    --device cuda \
    --json
```

JSON output reports — for each top-K hit — the matched image path, its
`(x, y, z)`, and the source database directory (i.e. the submap id).

Long-running RPC mode (this is how `vpr_bootstrap` keeps the model loaded
between queries from `server.py`):

```bash
uv run python vpr_detect.py \
    --databases /home/nvidia/Navi/maps/408/vpr/floor1 \
                /home/nvidia/Navi/maps/408/vpr/floor2 \
                /home/nvidia/Navi/maps/408/vpr/floor3 \
    --device cuda \
    --serve
# Then write one JSON request per line on stdin and read one JSON response per
# line from stdout. See NaviSkill/vpr_bootstrap.py for the wire format.
```

The set of `--databases` passed to `--serve` must all share the same
undistortion config (validated at startup against each
`database_build_info.json`); the live query image is preprocessed accordingly.

## Quick-Start Evaluation

Evaluation datasets are included in `datasets/`. Run these after `uv sync`:

### Cross-floor recognition (SALAD-indoor + color, best config)

```bash
uv run python eval/pipeline_3d.py \
    --method=salad-indoor \
    --database_folder=datasets/cross_floor_eval/database \
    --queries_folder=datasets/cross_floor_eval/queries \
    --no_labels --color_rerank --color_weight=5.0 \
    --num_preds_to_save=3 --log_dir=cross_floor_eval
```

Expected: **99.1% floor accuracy** (111/112 correct). Results in `logs/cross_floor_eval/<timestamp>/preds/`.

### Same-floor localization (per-floor with recall)

```bash
# Example: 11th floor, with geometric re-ranking
uv run python eval/pipeline_3d.py \
    --method=megaloc \
    --database_folder=datasets/floor_eval/11th-floor/database \
    --queries_folder=datasets/floor_eval/11th-floor/queries \
    --positive_dist_threshold=3 \
    --rerank --rerank_top_k=20 \
    --num_preds_to_save=3 --log_dir=floor_eval_11th
```

Expected: **96.4% R@1** for 11th floor. Replace `11th-floor` with `12th-floor`, `3rd-floor`, or `4th-floor`.

### Included datasets

| Dataset | DB Images | Query Images | Ground Truth | Purpose |
|---------|-----------|--------------|--------------|---------|
| `cross_floor_eval` | 59 (4 floors) | 112 (28/floor) | Floor label in filename | Cross-floor recognition |
| `floor_eval/11th-floor` | 15 | 28 | UTM coords in filename | Same-floor R@1 |
| `floor_eval/12th-floor` | 15 | 28 | UTM coords in filename | Same-floor R@1 |
| `floor_eval/3rd-floor` | 16 | 28 | UTM coords in filename | Same-floor R@1 |
| `floor_eval/4th-floor` | 13 | 28 | UTM coords in filename | Same-floor R@1 |

---

## Step 0: Database Construction

Before running the VPR pipeline, extract images from raw data into database and query folders.

### Supported input sources

| Source | Flag | Requirements | Pose Source |
|--------|------|-------------|-------------|
| ROS1 bag (.bag) | `--source=ros1` | `rosbags` (pip) | `frame_pose.txt` |
| ROS2 bag (.db3) | `--source=ros2` | ROS2 Humble (system) | Odometry topic |
| Image folder | `--source=folder` | None | `frame_pose.txt` |

### From ROS1 bag (SHARE SLAM S20 with fisheye undistortion)

```bash
python build_database.py --source=ros1 \
    --bag_path=data/all_2026-02-25-15-20-55.bag \
    --pose_file=data/frame_pose.txt \
    --image_topic=/camera_agent/img_left/compressed \
    --undistort_fisheye \
    --db_distance=1.0 --query_distance=0.5 \
    --output_dir=datasets/my_scene
```

### From ROS2 bag (ZED robot with odometry)

```bash
python build_database.py --source=ros2 \
    --bag_path=data/ \
    --pose_topic=/laser_odom_path \
    --image_topic=/zed/zed_node/rgb/color/rect/image \
    --db_distance=1.0 --query_distance=0.5 \
    --output_dir=datasets/my_scene
```

### From plain image folder

```bash
python build_database.py --source=folder \
    --image_dir=raw_images/ \
    --pose_file=poses.txt \
    --db_distance=1.0 --query_distance=0.5 \
    --output_dir=datasets/my_scene
```

### Pose file format

`frame_pose.txt` — one pose per line, space-separated:
```
# timestamp tx ty tz qx qy qz qw
1772004062.900 -0.000113 -0.000093 0.000036 0 0 0 1
1772004063.013  0.000015  0.000079 0.000091 0 0 0 1
```

Only `timestamp`, `tx`, `ty` are used (position for distance sampling). `tz` and quaternion are ignored.

### Distance sampling parameters

| Parameter | Default | Effect |
|-----------|---------|--------|
| `--db_distance` | 1.0m | Min distance between database images |
| `--query_distance` | 0.5m | Min distance between query images |

Images closer than `--db_distance` apart go to queries (or are skipped). Query images within 0.1m of any database image are excluded to avoid trivial matches.

### Fisheye undistortion

For POLYFISHEYE cameras (e.g., SHARE SLAM S20), add `--undistort_fisheye`:
- Uses built-in S20 calibration by default
- Or specify `--calibration_file=path/to/calibration.yaml`
- Output: perspective projection at `--out_width` x `--out_height` with `--fov` degrees FOV

### Output

```
output_dir/
├── database/
│   ├── @-1.234@5.678@1771717710792742774@.jpg
│   └── ...
└── queries/
    ├── @-1.300@5.700@1771717711462626774@.jpg
    └── ...
```

---

## Step 1-4: VPR Pipeline

### Cross-floor recognition (recommended for multi-floor buildings)

```bash
python eval/pipeline_3d.py \
    --method=salad-indoor \
    --database_folder=/path/to/database \
    --queries_folder=/path/to/queries \
    --no_labels \
    --color_rerank --color_weight=5.0 \
    --num_preds_to_save=3
```

### Same-floor localization (with geometric re-ranking)

```bash
python eval/pipeline_3d.py \
    --method=megaloc \
    --database_folder=/path/to/database \
    --queries_folder=/path/to/queries \
    --no_labels \
    --rerank --rerank_top_k=20 \
    --num_preds_to_save=3
```

### With ground-truth recall evaluation

Requires filenames in `@x_coord@y_coord@...@.jpg` format:

```bash
python eval/pipeline_3d.py \
    --method=megaloc \
    --database_folder=/path/to/database \
    --queries_folder=/path/to/queries \
    --positive_dist_threshold=3 \
    --rerank \
    --num_preds_to_save=3
```

## Input Format

### Image folders

Two folders: one for database images, one for query images. Supported formats: `.jpg`, `.jpeg`, `.png`.

### Filename convention (for ground-truth evaluation)

```
@<x_coordinate>@<y_coordinate>@<any_extra_info>@.jpg
```

Example: `@-12.330235@3.474244@1771927384407837007@.jpg`

The x and y coordinates are parsed for computing Euclidean distance between query and database positions. Any coordinate system works (UTM, local odometry, etc.) as long as database and queries share the same frame.

When `--no_labels` is used, filenames can be anything.

## Output

All outputs are saved under `logs/<log_dir>/<timestamp>/`:

```
logs/default/2026-02-25_10-43-02/
├── info.log                    # Run log with recall results
├── debug.log                   # Detailed debug log
├── rerank_scores.npy           # (num_queries, k) inlier counts [if --rerank]
├── queries_descriptors.npy     # (num_queries, 8448) [if --save_descriptors]
├── database_descriptors.npy    # (num_database, 8448) [if --save_descriptors]
└── preds/                      # [if --num_preds_to_save > 0]
    ├── 000.jpg                 # Side-by-side: query + top-N predictions
    ├── 000.txt                 # Paths + confidence scores
    ├── 001.jpg
    ├── 001.txt
    └── ...
```

### Prediction text file format

```
Query path:
/path/to/query.jpg

Predictions paths:
/path/to/pred0.jpg
/path/to/pred1.jpg
/path/to/pred2.jpg

Confidence (inlier count):
142, 85, 37
```

### Confidence interpretation

| Inlier Count | Interpretation |
|-------------|----------------|
| > 50 | Strong match — high confidence |
| 20 – 50 | Moderate — likely correct but verify |
| 5 – 20 | Weak — may be wrong |
| 0 – 5 | No valid match found in database |

## Workflow for New Data

### Step 1: Prepare images

Extract images from your data source (ROS bags, video, etc.) and organize into two folders. For distance-based sampling from ROS bags, see the extraction scripts in the parent repository.

### Step 2: Run the pipeline

```bash
# Quick test (no re-ranking)
python eval/pipeline_3d.py --database_folder=db/ --queries_folder=queries/ --no_labels --num_preds_to_save=3

# Production (with re-ranking for higher accuracy)
python eval/pipeline_3d.py --database_folder=db/ --queries_folder=queries/ --no_labels --rerank --num_preds_to_save=3
```

### Step 3: Inspect results

- Browse `logs/*/preds/*.jpg` for visual predictions
- Check `logs/*/preds/*.txt` for confidence scores
- Use `rerank_scores.npy` for downstream filtering (e.g., reject matches with < 20 inliers)

## Configuration Reference

| Parameter | Default | Description |
|-----------|---------|-------------|
| `--method` | megaloc | `megaloc` (outdoor/geometry) or `salad-indoor` (indoor/appearance) |
| `--device` | cuda | cuda or cpu |
| `--batch_size` | 4 | Batch size for descriptor extraction |
| `--num_workers` | 4 | DataLoader workers |
| `--rerank` | off | Enable SuperPoint+LightGlue geometric re-ranking |
| `--rerank_top_k` | 20 | Candidates to re-rank per query |
| `--color_rerank` | off | Enable HSV color histogram re-scoring |
| `--color_weight` | 0.5 | Alpha for color scoring: `score = base * (1 + α * color_sim)` |
| `--no_labels` | off | Skip ground-truth recall computation |
| `--positive_dist_threshold` | 25 | Distance (m) for correct match |
| `--recall_values` | 1 5 10 20 | R@k values to compute |
| `--num_preds_to_save` | 0 | Save top-N visualizations (0=disabled) |
| `--save_descriptors` | off | Save descriptor numpy arrays |
| `--image_size` | None | Resize images before descriptor extraction |

## Performance Benchmarks

Tested on NVIDIA RTX 6000 Ada (48GB VRAM), 32-thread CPU:

| Stage | Speed | GPU Usage | VRAM |
|-------|-------|-----------|------|
| MegaLoc descriptors (batch=4) | ~9 img/s | 93-96% | ~3 GB |
| FAISS search (59 DB images) | < 1ms | 0% | 0 |
| LightGlue re-ranking (20 candidates) | ~0.4s/query | ~50% | ~1 GB |
| Prediction saving | ~20 img/s | 0% | 0 |

## Model Details

### MegaLoc (Stage 1 — outdoor/geometry)

- Backbone: DINOv2 ViT-L/14 (frozen foundation model)
- Aggregator: SALAD-style learned aggregation
- Descriptor: 8448-dimensional L2-normalized vector
- Training: Large-scale street-view geo-localization
- Strength: Robust to lighting/weather changes, strong geometric matching
- Weakness: Color-invariant — cannot distinguish floors with identical geometry
- Paper: MegaLoc (2025)

### SALAD-indoor (Stage 1 — indoor/appearance)

- Backbone: DINOv2 ViT-B/14 (frozen foundation model)
- Aggregator: SALAD (Sinkhorn optimal transport pooling)
- Descriptor: 8448-dimensional vector
- Training: Base SALAD on street-view, then indoor fine-tuning with contrastive loss
- Strength: Retains appearance sensitivity (wall colors, floor materials, signage)
- Best for: Cross-floor recognition where floors share geometry but differ in color
- Source: Indoor-VPR (Chiavassa et al.)

### Color Re-scoring (Stage 4)

- Computes 2D hue-saturation histograms in HSV color space
- Masks out extreme brightness pixels (unreliable hue)
- Bhattacharyya similarity between query and candidate histograms
- Combined score: `final = base_score * (1 + alpha * color_similarity)`
- Alpha=5.0 recommended for cross-floor tasks

### SuperPoint (Stage 3)

- CNN-based keypoint detector + descriptor extractor
- Outputs up to 1024 keypoints per image (configurable)
- 256-dimensional descriptor per keypoint
- Self-supervised training on synthetic + real data
- Paper: SuperPoint (CVPR 2018 Workshop)

### LightGlue (Stage 3)

- Transformer-based feature matcher
- Cross-attention between two sets of keypoints
- Learns to reject non-matchable keypoints
- Faster and more accurate than SuperGlue
- Paper: LightGlue (ICCV 2023)
