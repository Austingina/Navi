# SAM3 in Navi

This directory vendors the source package, scripts, export tools, examples and
sample assets from the local SAM3 checkout used for the Navi migration. Upstream:
https://github.com/facebookresearch/sam3. The source checkout did not contain Git
history, so an exact upstream commit is not asserted. See LICENSE for the original
SAM License. Existing local changes are preserved. Navi additionally replaces
machine-specific input/checkpoint defaults with explicit CLI arguments, resolves
the BPE vocabulary relative to this source tree, and uses relative output paths.

Checkpoints, TensorRT/ONNX artifacts, virtual environments, caches and machine
diagnostic logs are excluded. COLCON_IGNORE prevents colcon from treating this
vendored Python project as a ROS package.

The navigation node defaults to TensorRT engines exported from Transformers SAM3;
the PyTorch backend remains available. Its adapter
lives in ros2_ws/src/navi_semantic_perception; this source tree preserves the
original SAM3 implementation and supporting tools for development. Switching to
the native Meta API is a separate backend change with additional dependencies.

Runtime installation: [envs/sam3/README.md](../envs/sam3/README.md). The environment is created inside
Navi at envs/sam3/.venv. Non-weight model assets are tracked in the ROS package at
models/sam3 and installed by colcon. The model.safetensors checkpoint and generated ONNX/TensorRT engines are
stored in the git-ignored repository directory checkpoints/sam3/ by default.
