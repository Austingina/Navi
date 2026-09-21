# HuggingFace SAM 3 — ONNX Export & TensorRT Inference

Export the HuggingFace SAM 3 model to ONNX format and build optimized
TensorRT engines for low-latency inference with multi-text-prompt support.

## Architecture

The model is split into **two ONNX graphs** so that the expensive vision
encoder runs only once per image while supporting an arbitrary number of
text prompts (N):

```
                    ┌─────────────────────┐
  pixel_values      │   VisionEncoder     │    fpn_feat_0/1/2
  [1,3,1008,1008] ──│   (ONNX Graph 1)    │──► fpn_pos_2
                    │   Static shape      │    [1, 256, H, W]
                    └─────────────────────┘
                                │
                     Python: repeat to batch=N
                                │
                    ┌─────────────────────┐
  input_ids         │   DetectorBody      │    pred_masks  [N,200,288,288]
  attention_mask  ──│   (ONNX Graph 2)    │──► pred_boxes  [N,200,4]
  [N, L]            │   Dynamic batch=N   │    pred_logits [N,200]
                    └─────────────────────┘    presence    [N,1]
                                │
                     Python: postprocess + NMS
```

**VisionEncoder** wraps `Sam3VisionModel` (ViT backbone + FPN neck).
**DetectorBody** wraps text encoder + DETR encoder/decoder + mask decoder.

## Setup

A local copy of the HuggingFace SAM 3 checkpoint is expected (the default
path is `sam3/sam3/`). You can change it with `--model`.

### Desktop / Server (x86_64)

Requires Python 3.10+ and a CUDA-capable GPU. Using [uv](https://docs.astral.sh/uv/)
for environment management:

```bash
cd sam3

# Create venv and install sam3 with export dependencies
uv venv --python 3.12
source .venv/bin/activate
uv pip install torch --index-url https://download.pytorch.org/whl/cu124
uv pip install -e ".[export]"
```

> **Note:** PyTorch must be installed separately with the correct CUDA variant
> for your system before `pip install -e ".[export]"`. The `tensorrt-cu12-*`
> packages require a matching CUDA 12.x toolkit on the host.

### Jetson AGX Orin (aarch64 / JetPack 6.x)

On Jetson, CUDA and TensorRT are pre-installed by JetPack. The
`tensorrt-cu12-*` pip packages are x86_64-only and must **not** be installed.

```bash
cd sam3

# Ensure MAX power mode for best performance
sudo nvpmodel -m 0 && sudo jetson_clocks

# Create venv with access to system-installed packages (TensorRT, CUDA)
# Python 3.10 or 3.12 — use whichever your JetPack ships (check: python3 --version)
uv venv --python 3.10 --system-site-packages
source .venv/bin/activate

# Install PyTorch from the Jetson AI Lab wheel index
# NOTE: archiconda / Python 3.7 will NOT work — the index only has cp310+ wheels
# Pin torch<=2.8 for JetPack 6.1 (CUDA 12.6); newer builds need a newer driver.
# Pin numpy<2 because the Jetson torch wheel was compiled against NumPy 1.x.
uv pip install --no-cache-dir "torch==2.8.0" "torchvision==0.23.0" "numpy<2" \
    --index-url https://pypi.jetson-ai-lab.io/jp6/cu126

# Install sam3 with Jetson export dependencies (no TensorRT pip packages)
uv pip install -e ".[export-jetson]"
```

Verify the system TensorRT is accessible:

```bash
python -c "import tensorrt; print(tensorrt.__version__)"
```

> **Note:** The `--system-site-packages` flag is required so the venv can
> see the JetPack-provided TensorRT and CUDA Python bindings. On Jetson,
> CPU and GPU share a unified memory pool — see [Memory on Jetson](#memory-on-jetson)
> below for details.

## Quick Start

All commands run from the `sam3/` directory:

```bash
# Step 1 — Export ONNX (runs on CPU, ~30s)
python export_hf/export_onnx.py

# Step 2 — Build TensorRT engines (requires GPU, ~2-3 min)
python export_hf/build_trt.py

# Step 3 — Run benchmark
python export_hf/trt_infer.py --engine_dir export_hf/trt_engines
```

## Step-by-Step Guide

### 1. Export to ONNX

```bash
python export_hf/export_onnx.py \
    --model /path/to/sam3/checkpoint \
    --output-dir export_hf/onnx_weights \
    --opset 17
```

| Argument       | Default                   | Description                          |
|----------------|---------------------------|--------------------------------------|
| `--model`      | `sam3/sam3`               | HuggingFace model name or local path |
| `--output-dir` | `export_hf/onnx_weights`  | Where to save the `.onnx` files      |
| `--device`     | `cpu`                     | Device for tracing (cpu recommended) |
| `--opset`      | `17`                      | ONNX opset version                   |

This produces two files:

| File                  | Size   | Description                       |
|-----------------------|--------|-----------------------------------|
| `vision_encoder.onnx` | ~1.8 GB | ViT + FPN (static shape)         |
| `detector_body.onnx`  | ~1.5 GB | Text + DETR + Mask (dynamic N)   |

### 2. Build TensorRT Engines

```bash
python export_hf/build_trt.py \
    --onnx-dir export_hf/onnx_weights \
    --engine-dir export_hf/trt_engines \
    --max-prompts 8
```

Both engines are built in **FP16** (matching PyTorch's BF16 autocast precision).

| Argument        | Default                  | Description                                      |
|-----------------|--------------------------|--------------------------------------------------|
| `--onnx-dir`    | `export_hf/onnx_weights` | Directory containing the ONNX files              |
| `--engine-dir`  | `export_hf/trt_engines`  | Where to save the `.plan` engine files            |
| `--max-prompts` | `8`                      | Max text prompts (dynamic batch upper bound)      |
| `--verbose`     | off                      | Print TensorRT builder logs                       |

**Note:** The engines are tied to your specific GPU and TensorRT version.
If you change either, rebuild the engines.

### 3. Run Inference / Benchmark

```bash
python export_hf/trt_infer.py \
    --engine_dir export_hf/trt_engines \
    --input_dir /path/to/images \
    --output_dir hf_trt_outputs \
    --model_name /path/to/tokenizer/and/processor \
    --confidence_threshold 0.6 \
    --max_images 100
```

| Argument                   | Default        | Description                                |
|----------------------------|----------------|--------------------------------------------|
| `--engine_dir`             | `export_hf/trt_engines` | Directory containing `.plan` files |
| `--model_name`             | `sam3/sam3`    | Model path (only for tokenizer/processor)  |
| `--input_dir`              | *(see code)*   | Directory of input images                  |
| `--output_dir`             | `hf_trt_outputs` | Where to save visualisations + JSON     |
| `--classes`                | *(4 defaults)* | Space-separated list of class names        |
| `--confidence_threshold`   | `0.6`          | Minimum detection confidence               |
| `--intra_nms_threshold`    | `0.5`          | Intra-class box NMS IoU threshold          |
| `--nms_iou_threshold`      | `0.8`          | Cross-class mask NMS IoU threshold         |
| `--nms_proximity_threshold`| `20.0`         | Suppress boxes whose centres are closer    |
| `--nms_min_mask_ratio`     | `0.0001`       | Discard masks smaller than this fraction   |
| `--max_images`             | `500`          | Maximum number of images to process        |
| `--gpu`                    | `0`            | GPU device index                           |
| `--no_vis`                 | off            | Skip saving visualisation images           |

### 4. Use the TRT Inference Class Directly

```python
from export_hf.trt_infer_helper import HFSam3TRTInference
from PIL import Image

engine = HFSam3TRTInference(
    engine_dir="export_hf/trt_engines",
    model_name="sam3/sam3",          # for tokenizer / image processor
    confidence_threshold=0.6,
    device="cuda:0",
    use_gpu_preprocess=True,         # nvJPEG GPU decode (default)
)

# Option A: from PIL image
image = Image.open("photo.jpg").convert("RGB")
result = engine.predict(image, ["car", "person", "tree"])

# Option B: from raw JPEG bytes (fastest — avoids PIL entirely)
with open("photo.jpg", "rb") as f:
    raw_bytes = f.read()
result = engine.predict_from_bytes(raw_bytes, (1280, 1280), ["car", "person", "tree"])

# Pre-cache text tokens when the class list doesn't change between images
engine.cache_text_inputs(["car", "person", "tree"])

# result = {
#     "results_by_class": {
#         "car":    {"masks": Tensor, "boxes": Tensor, "scores": Tensor, "count": int},
#         "person": {...},
#     },
#     "total_objects": int,
#     "classes_detected": ["car", "person"],
# }
```

## Performance

### Desktop (RTX 4090)

Measured on 500 images (1280×1280 JPEG) on an RTX 4090 (24 GB), TensorRT 10.16.

#### Inference Latency

| Configuration | Prompts | Vision | Detector | Inference Latency | Inference FPS |
|---|---|---|---|---|---|
| HF PyTorch (BF16 autocast) | 4 | 69.2 ms | 39.6 ms | 113.3 ms | 8.8 |
| TRT FP16 (baseline) | 4 | 37.9 ms | 18.3 ms | 57.7 ms | 17.3 |
| **TRT FP16 + nvJPEG + async** | **4** | **31.0 ms** | **18.2 ms** | **53.5 ms** | **18.7** |

Inference latency measures only the GPU compute path (decode + vision + repeat +
detector). It excludes I/O and visualization.

#### End-to-End Throughput

| Configuration | With Vis | Without Vis |
|---|---|---|
| TRT FP16 (baseline) | 12.6 it/s (39.5s) | ~14.3 it/s (~35s) |
| **TRT FP16 + nvJPEG + async** | **16.2 it/s (30.8s)** | **~18.7 it/s (~26.7s)** |
| **Improvement** | **+29%** | **+31%** |

The throughput improvement comes from three optimisations:

- **Async I/O prefetch** — a background thread reads the next image's raw
  bytes while the GPU processes the current one, hiding ~12 ms of disk I/O
  behind GPU compute.
- **nvJPEG GPU decode** — `torchvision.io.decode_jpeg(device="cuda")` decodes
  JPEG directly to GPU memory, followed by GPU resize and normalise. This
  replaces the CPU-bound Sam3Processor image pipeline (PIL decode → CPU resize →
  CPU normalise → H2D copy).
- **Text token caching** — text prompts are tokenised once and reused for
  every image, eliminating ~0.4 ms/image of repeated tokenisation.

When visualisation is enabled, the background thread also pre-decodes the
image via `cv2.imdecode` from the in-memory bytes, so the visualisation
path avoids a second disk read.

#### VRAM Usage (Desktop)

| Configuration | At Engine/Model Load | Peak During Inference |
|---|---|---|
| HF PyTorch (BF16 autocast) | 4,274 MiB | 8,464 MiB |
| TRT FP32 | 21,966 MiB | 22,886 MiB |
| **TRT FP16** | **4,120 MiB** | **5,046 MiB** (4 prompts) |

Driver-level VRAM measured with pynvml. TRT FP16 uses less memory at load
than PyTorch (no Python-side weight tensors), and peaks lower because the
TRT execution context workspace is pre-allocated at engine load for the
maximum batch profile (`--max-prompts`), while PyTorch allocates activations
on-the-fly during the forward pass.

The TRT workspace is fixed regardless of how many prompts you actually use;
the VRAM increase from 1→4 prompts (~564 MiB) comes from PyTorch tensors
created outside the engines (`.repeat()` on FPN features, detector output
buffers, and postprocessing temporaries).

**Why TRT FP32 uses ~22 GB.** Three factors compound:

1. **All intermediate activations are FP32 (4 bytes), not BF16 (2 bytes).**
   Model weights are FP32 in both PyTorch and TRT, so they cost the same
   (~3.2 GB). The difference is in the *activations* — the feature maps,
   attention matrices, and decoder hidden states that flow through the
   network during inference. PyTorch under `torch.autocast(dtype=bfloat16)`
   casts weights to BF16 on-the-fly and produces BF16 intermediate tensors,
   halving activation memory. TRT FP32 keeps everything at full width.

2. **TRT pre-allocates workspace for the maximum dynamic shape.**  The
   detector engine is built with dynamic batch 1–8 (`--max-prompts 8`).
   TRT allocates device memory for the *worst-case* profile at context
   creation time, not the actual batch used at runtime. At batch=8 in FP32,
   a single FPN feature tensor (`fpn_feat_0` at `[8, 256, 288, 288]`) is
   618 MB — and there are hundreds of such intermediates across the ViT
   layers and DETR decoder.

3. **Engine files carry more than weights.** The `.plan` files embed the
   optimized computation graph, kernel selection tables, and pre-compiled
   CUDA kernels alongside the weights. Two engines (vision 1.7 GB +
   detector 1.4 GB) plus their execution contexts and workspaces add up
   quickly. PyTorch only stores the raw weight tensors in GPU memory.

Building with `--fp16` addresses all three: activations drop to 16 bits,
the pre-allocated workspace shrinks by ~2×, and the engine files are
roughly half the size — which is why TRT FP16 fits in ~5 GB vs ~23 GB
for FP32.

### Jetson AGX Orin

Measured on 100 images (1280×1280 JPEG) on an AGX Orin 64 GB,
JetPack 6.1 (L4T R36.4, CUDA 12.6), TRT FP16, torch 2.8.0,
MAX power mode (`nvpmodel -m 0 && jetson_clocks`).

**Power modes.** The Orin has multiple power profiles that trade
performance for power/thermals. `nvpmodel -m 0` selects MAXN (60 W, all
12 CPU cores, max GPU clock ~1.3 GHz). The default after boot is
typically a 30 W mode (~0.9 GHz GPU, 8 cores), which is roughly
**40–50 % slower** for GPU-bound workloads like this pipeline. A 15 W
mode drops to ~30–40 % of MAXN throughput. `jetson_clocks` pins all
clocks at their ceiling within the chosen profile — without it, the
governor scales dynamically and can introduce latency spikes during
ramp-up. Together the two commands give the best and most consistent
benchmark numbers.

To revert after benchmarking:

```bash
sudo jetson_clocks --restore   # re-enable dynamic clock scaling
sudo nvpmodel -m 2             # switch back to 30 W (or your preferred mode)
```

A reboot also resets everything to defaults. Leaving MAXN on is safe but
runs hotter at idle, which matters in enclosed or battery-powered
deployments.

#### Inference Latency (Jetson)

| Stage | Latency | % of total |
|---|---|---|
| Vision encoder | 370.5 ms | 57.9% |
| Detector | 187.5 ms | 29.3% |
| GPU decode (nvJPEG) | 15.9 ms | 2.5% |
| Vision repeat | 8.5 ms | 1.3% |
| Postprocess + NMS | 10.2 ms | 1.6% |
| **Total inference** | **582 ms** | **1.72 FPS** |
| Visualization | 46.4 ms | 7.2% |

The Orin is roughly **10× slower** than the RTX 4090 for the
vision-encoder-dominated pipeline, consistent with the difference in GPU
compute throughput.

#### Memory Usage (Jetson)

On Jetson, CPU and GPU share a **unified memory pool** — there is no
dedicated VRAM. NVML (`pynvml`) is not available on Tegra; the benchmark
reports memory via `/proc/meminfo` and process RSS instead.

| Metric | At Engine Load | Peak (4 prompts) |
|---|---|---|
| Process RSS | 1,056 MiB | 1,346 MiB |
| PyTorch reserved | 0 MiB | 918 MiB |
| System memory used (total) | 25,015 MiB | 26,201 MiB |

Estimated total inference footprint breakdown:

| Component | Size |
|---|---|
| TRT vision encoder engine | ~870 MiB |
| TRT detector body engine | ~743 MiB |
| TRT execution workspaces | ~500–900 MiB |
| PyTorch tensors (peak) | ~918 MiB |
| Python / library overhead | ~300 MiB |
| **Estimated total** | **~3.3–3.7 GB** |

| Jetson SKU | Unified Memory | Estimated Inference Peak | Headroom |
|---|---|---|---|
| Orin 64 GB | 63 GB | ~3.7 GB | Plenty — `--max-prompts 8` is fine |
| Orin 32 GB | 32 GB | ~3.7 GB | Fine for inference; engine **build** peaks ~16 GB — close other processes |

The TRT engine builder is the most memory-hungry phase (~16 GB). On the
32 GB SKU, stop other GPU workloads before building engines, or build on a
desktop and copy the `.plan` files to the Jetson (engines are
architecture-specific; the Orin GPU is `sm_87`).

## File Overview

```
export_hf/
├── __init__.py
├── hf_vision_wrapper.py     # VisionEncoder ONNX wrapper
├── hf_detector_wrapper.py   # DetectorBody ONNX wrapper (text + DETR + mask)
├── export_onnx.py           # Export script → 2 ONNX files
├── build_trt.py             # Build TRT engines from ONNX
├── trt_infer_helper.py      # TRT engine wrapper, GpuPreprocessor, postprocessing
├── trt_infer.py             # Benchmark with async I/O, NMS, timing, visualisation
└── README.md                # This file
```

### Key classes in `trt_infer_helper.py`

| Class | Purpose |
|---|---|
| `TRTEngine` | Thin wrapper around a single TensorRT engine (load, bind, infer) |
| `GpuPreprocessor` | nvJPEG decode + GPU resize/normalise (replaces Sam3Processor for images) |
| `HFSam3TRTInference` | Orchestrates the two engines with text token caching and postprocessing |

### Key classes in `trt_infer.py`

| Class | Purpose |
|---|---|
| `PrefetchLoader` | Background-thread file reader with optional cv2 pre-decode for visualisation |
| `TRTBenchmarkProcessor` | Wraps `HFSam3TRTInference` with per-stage timing |

## Troubleshooting

**ONNX export fails with `create_causal_mask` or `create_bidirectional_mask`
errors** — The export script patches these HF masking utilities automatically.
Make sure you are running `export_onnx.py` and not calling `torch.onnx.export`
directly on the raw model.

**TRT engine build fails with OOM** — The DetectorBody engine uses up to
16 GB of GPU memory during optimisation. Close other GPU processes or reduce
`--max-prompts`.

**Different detection counts between PyTorch and TRT** — Ensure the ONNX
export ran on CPU (`--device cpu`, the default) to avoid GPU non-determinism
during tracing. Small FP16 numerical differences are expected but should not
affect detection counts at practical confidence thresholds.

**`Int64 binding` warnings during TRT build** — These are benign. TensorRT
internally handles the int64-to-int32 downcast for `input_ids` and
`attention_mask`.

**`import tensorrt` fails on Jetson** — Make sure the venv was created with
`--system-site-packages` so it can see the JetPack-provided TensorRT. Do
**not** `pip install tensorrt-cu12-*` — those wheels are x86_64-only.

**Slow inference on Jetson** — Verify the power mode is set to MAX:
`sudo nvpmodel -m 0 && sudo jetson_clocks`. The default power mode
throttles both CPU and GPU clocks significantly.

**`NVIDIA driver too old (found version 12060)`** — The system CUDA driver
on JetPack 6.1 is 12.6. Newer torch builds (≥2.9) from the Jetson AI Lab
index may require a higher driver version. Pin `torch==2.8.0` and
`torchvision==0.23.0` for JetPack 6.1 compatibility.

**`pip install` says "Defaulting to user installation"** — The venv was
created by `uv` which does not install pip inside it. Use `uv pip install`
instead of bare `pip install`, or bootstrap pip first with
`uv pip install pip`.

**`No matching distribution found for torch` from the old NVIDIA index** —
The legacy URL `developer.download.nvidia.com/compute/redist/jp/v61` uses a
non-standard directory layout that pip cannot resolve. Use the Jetson AI Lab
index instead: `--index-url https://pypi.jetson-ai-lab.io/jp6/cu126`.

**NumPy ABI mismatch / `compiled using NumPy 1.x`** — The Jetson torch 2.8.0
wheel was compiled against NumPy 1.x. Pin `numpy<2` in the venv to avoid
the crash.

**`pynvml not available` on Jetson** — This is expected. NVML does not
support Jetson's Tegra integrated GPU. The benchmark automatically falls
back to `/proc/meminfo` for unified-memory stats. You can safely ignore
the `pynvml` deprecation warning emitted by torch's internal import.

## ROS 2 Real-Time Node

`ros2_trt_node.py` is a ROS 2 node that subscribes to a live camera topic,
runs the TRT SAM 3 pipeline on each frame, and publishes annotated images
for real-time visualization in RViz2.

### Published Topics

| Topic                      | Type                      | Description                                      |
|----------------------------|---------------------------|--------------------------------------------------|
| `/sam3/detections/image`   | `sensor_msgs/msg/Image`   | Annotated image with masks, boxes, and labels    |
| `/sam3/detections/json`    | `std_msgs/msg/String`     | JSON with per-frame detections (class, score, bbox) |

### Quick Start

```bash
cd sam3
source .venv/bin/activate
source /opt/ros/humble/setup.bash

# Start the detector node (default classes: elevator, fire ext box, etc.)
python export_hf/ros2_trt_node.py \
    --engine_dir export_hf/trt_engines \
    --model_name sam3/sam3

# Or specify custom text prompts — more descriptive prompts work best
python export_hf/ros2_trt_node.py \
    --engine_dir export_hf/trt_engines \
    --model_name sam3/sam3 \
    --classes "robot" "ground" "box"
```

### RViz2 Visualization

1. In a second terminal, open RViz2: `rviz2`
2. Click **Add** → **By topic** → expand `/sam3/detections/image` → select **Image**
3. The annotated image panel appears with live masks, bounding boxes, and labels

> **Tip:** Use the **Image** display type (not Camera). The Camera display
> requires a paired CameraInfo topic and TF, which the detection topic does
> not publish.

### Arguments

| Argument                    | Default                                 | Description                          |
|-----------------------------|-----------------------------------------|--------------------------------------|
| `--engine_dir`              | `export_hf/trt_engines`                 | TRT engine directory                 |
| `--model_name`              | `sam3/sam3`                             | HF model path (for processor)       |
| `--image_topic`             | `/camera/camera/color/image_raw`        | ROS 2 image topic to subscribe to   |
| `--classes`                 | elevator, fire ext box, fire sign, exit | Text prompts for detection           |
| `--confidence_threshold`    | `0.6`                                   | Minimum detection confidence         |
| `--gpu`                     | `0`                                     | CUDA device index                    |
| `--intra_nms_threshold`     | `0.5`                                   | Intra-class NMS IoU threshold        |
| `--nms_iou_threshold`       | `0.8`                                   | Cross-class NMS IoU threshold        |
| `--nms_proximity_threshold` | `20.0`                                  | Cross-class NMS proximity (pixels)   |

The node drops stale frames automatically — if inference is slower than
the camera rate, only the most recent frame is processed.
