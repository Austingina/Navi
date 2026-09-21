# SAM3 model assets

Model configuration, tokenizer and image processor files used by Navi are tracked
here so a clone includes all non-weight assets. They were copied from the working
local SAM3 deployment. The original model is Meta's facebook/sam3:
https://huggingface.co/facebook/sam3. Its license is preserved in LICENSE.

The pinned checkpoint source is recorded in envs/sam3/prepare_assets.py. The
model.safetensors file is downloaded or imported separately into
<Navi>/checkpoints/sam3/ by default; the repository root .gitignore excludes it.
