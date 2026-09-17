# Setup on Jetson Thor / JetPack 7

`export_hf/README.md`'s Jetson section targets **AGX Orin / JetPack 6.x / CUDA 12.6**.
None of those commands work as written on Thor. This file records what actually
works, verified on:

| | |
|---|---|
| Board | Jetson Thor (`aarch64`), 122 GB unified memory |
| JetPack | 7.1-b112 — L4T R38.4.0, kernel `6.8.12-rt-tegra` |
| CUDA | 13.0 (`/usr/local/cuda-13.0`) |
| TensorRT | 10.13.3.9 (`+cuda13.0`, apt, in `/usr/lib/python3.12/dist-packages`) |
| Python | 3.12.3 |
| ROS 2 | Jazzy |

## Deviations from `export_hf/README.md`

| README says | Reality on Thor |
|---|---|
| `--index-url https://pypi.jetson-ai-lab.io/jp6/cu126` | There is **no `jp7` user** on that devpi server (only `jp6/cu{126,128,129}`, `sbsa/cu130`, `sbsa/dev`). |
| use the Jetson AI Lab wheel index | Its `sbsa/cu130` torch wheel links `libnvpl_lapack_lp64_gomp.so.0`, which JetPack 7 does **not** ship → `ImportError` on `import torch`. Use `download.pytorch.org/whl/cu130` instead; those wheels bundle NVPL and list `sm_110` (Thor) in `torch.cuda.get_arch_list()`. |
| `torch==2.8.0` / `torchvision==0.23.0` | Those are CUDA 12.6 builds. Use `torch==2.13.0+cu130` / `torchvision==0.28.0+cu130`. |
| `uv venv --python 3.10` | Use 3.12 — that is where JetPack 7 installs the TensorRT bindings. |
| `transformers>=4.52` (pyproject) | Too low. `transformers.models.sam3` does **not exist in any 4.x** release (4.x ends at 4.57.6); SAM 3 is 5.x-only. Needs **`transformers>=5.0`**. |
| `source /opt/ros/humble/setup.bash` | Jazzy here. |

## Install

```bash
cd ~/Navi/sam3

# venv MUST use --system-site-packages so JetPack's TensorRT is visible
uv venv --python 3.12 --system-site-packages
python -c "import tensorrt" # via .venv/bin/python -> 10.13.3.9

# torch/torchvision from pytorch.org cu130 (SBSA aarch64), NOT jetson-ai-lab.
# No --extra-index-url here: uv gives extra indexes priority over --index-url
# and would silently install the CPU-only PyPI aarch64 wheel (torch==2.9.1+cpu).
uv pip install "torch==2.13.0+cu130" "torchvision==0.28.0+cu130" \
    --index-url https://download.pytorch.org/whl/cu130

# rest of the deps; numpy pinned <2 to stay ABI-compatible with ROS cv_bridge
uv pip install -e ".[export-jetson]" "transformers>=5.0" einops "numpy<2"
```

### Throttle every heavy step — this board hard-resets under full load

`systemd` holds `/dev/watchdog0` with `RuntimeWatchdogUSec=2min`, and
`kernel.panic=0` + `panic_on_oops=1` means a hang never auto-reboots — the
hardware watchdog does, silently. An unthrottled `uv pip install` (14 cores
pegged unpacking 4.5 GB to NVMe on an RT kernel) reproducibly triggers it:
three hard resets in 25 minutes, no logs, no thermal event, no OOM.
`nvme smart-log` shows `unsafe_shutdowns: 264 / power_cycles: 268`, so the
fragility predates this project.

Prefix long-running commands with:

```bash
UV_CONCURRENT_DOWNLOADS=2 UV_CONCURRENT_INSTALLS=2 UV_CONCURRENT_BUILDS=1 \
UV_HTTP_TIMEOUT=300 nice -n 19 ionice -c3 uv pip install ...
```

With that, the same install completes without a reset. Do not use
`-j$(nproc)` for native builds; cap at `-j4`.

Persistent journald was enabled so the next reset leaves evidence
(`/etc/systemd/journald.conf.d/99-persistent.conf`, capped at 500 MB) —
inspect with `journalctl -b -1 -k`.

## Verified working

```
torch 2.13.0+cu130   cuda 13.0   cuda.is_available() True   NVIDIA Thor
torchvision 0.28.0+cu130      tensorrt 10.13.3.9
transformers 5.16.1  (transformers.models.sam3 present,
                      masking_utils.create_bidirectional_mask present)
onnx 1.22.0   onnxruntime 1.29.0   opencv-python 4.11.0.86
numpy 1.26.4  timm 1.0.29  einops 0.8.2
export_hf.export_onnx / export_hf.trt_infer_helper import cleanly
rclpy + cv_bridge import and round-trip inside the venv with
  `source /opt/ros/jazzy/setup.bash`  (pip cv2 4.11 coexists with system 4.6)
```

## Known gaps

**1. The checkpoint is gated.** `facebook/sam3` is `gated: manual` — even
`config.json` returns 401 anonymously, and nothing is cached on this machine.
Before `export_onnx.py` can run, someone must request access at
<https://huggingface.co/facebook/sam3>, wait for approval, then:

```bash
.venv/bin/hf auth login      # paste an access token
```

**2. `import sam3` fails: `No module named 'decord'`.** Import chain is
`sam3/__init__.py` → `model_builder` → `sam1_task_predictor` →
`sam3_tracker_base` → `train/data/collator` → `train/data/sam3_image_dataset.py:18`
(`from decord import cpu, VideoReader`, a hard top-level import in a
*training* module). Neither `decord` nor `eva-decord` publishes an
`aarch64` Linux wheel, so it needs a source build (ffmpeg dev + cmake).

This does **not** block the TensorRT path: `export_onnx.py` goes through
`transformers.AutoModel`, and `export_hf.*` imports fine without it. Only the
PyTorch reference path (`sam3.build_sam3_image_model`) is affected.

**3. `export_onnx.py --model` default is a stale absolute path**
(`--model /path/to/hf-sam3`, `export_onnx.py`).
Always pass `--model` explicitly.

## Remaining steps (blocked on the checkpoint)

```bash
cd ~/Navi/sam3 && source .venv/bin/activate

nice -n 19 ionice -c3 python export_hf/export_onnx.py \
    --model facebook/sam3 --output-dir export_hf/onnx_weights --device cpu

# ~16 GB peak and minutes of saturated GPU — the highest-risk step for a
# watchdog reset. Consider dropping out of MAXN first.
nice -n 19 ionice -c3 python export_hf/build_trt.py \
    --onnx-dir export_hf/onnx_weights --engine-dir export_hf/trt_engines \
    --max-prompts 4

python export_hf/ros2_trt_node.py \
    --engine_dir export_hf/trt_engines --model_name facebook/sam3 \
    --image_topic /camera/camera/color/image_raw
```

Engines are tied to this GPU (Thor, `sm_110`) and to TensorRT 10.13.3.9;
rebuild if either changes.
