"""Prepare local SAM3 checkpoint weights using model assets tracked in Navi."""

import argparse
import json
import os
import shutil
from pathlib import Path


REPO_ID = 'facebook/sam3'
REVISION = '3c879f39826c281e95690f02c7821c4de09afae7'
ASSET_DIR = (Path(__file__).resolve().parents[2]
             / 'ros2_ws/src/navi_semantic_perception/models/sam3')
DEFAULT_CHECKPOINT_DIR = Path(__file__).resolve().parents[2] / 'checkpoints/sam3'


def checkpoint_directory():
    """Use the same repository-local weight location as the ROS node."""
    return Path(os.environ.get('SAM3_CHECKPOINT_DIR', DEFAULT_CHECKPOINT_DIR)).expanduser()


def require_files(directory, names):
    """Report incomplete assets before copying or loading them."""
    missing = [str(directory / name) for name in names if not (directory / name).is_file()]
    if missing:
        raise FileNotFoundError('Missing SAM3 assets: ' + ', '.join(missing))


def copy_file(source, destination):
    """Copy a weight file without truncating the destination if interrupted."""
    if destination.exists() and source.samefile(destination):
        return
    destination.parent.mkdir(parents=True, exist_ok=True)
    partial = destination.with_name(destination.name + '.partial')
    shutil.copy2(source, partial)
    partial.replace(destination)


def import_local(source, model_dir):
    """Accept a safetensors file, a weight directory, or an older deployment root."""
    candidates = (source, source / 'model.safetensors',
                  source / 'checkpoints/sam3/model.safetensors')
    checkpoint = next((path for path in candidates if path.is_file()), None)
    if checkpoint is None:
        raise FileNotFoundError(f'No model.safetensors checkpoint found at {source}')
    if checkpoint.suffix != '.safetensors':
        raise ValueError('The navigation backend requires HF .safetensors weights, not sam3.pt')
    copy_file(checkpoint, model_dir / 'model.safetensors')
    return {'source': 'local', 'path': str(checkpoint)}


def download_assets(model_dir):
    """Download just the checkpoint; configuration and tokenizer already ship in Git."""
    from huggingface_hub import hf_hub_download

    hf_hub_download(repo_id=REPO_ID, revision=REVISION, filename='model.safetensors',
                    local_dir=str(model_dir))
    return {'source': 'huggingface', 'repo_id': REPO_ID, 'revision': REVISION}


def validate_assets(model_dir):
    """Check bundled non-weight assets and the external tensor archive without a GPU."""
    from safetensors import safe_open
    from transformers import AutoImageProcessor, AutoTokenizer, Sam3Config

    require_files(model_dir, ('model.safetensors',))
    require_files(ASSET_DIR, ('config.json',))
    processor = ASSET_DIR / 'processor'
    require_files(processor, (
        'preprocessor_config.json', 'tokenizer.json', 'tokenizer_config.json'))
    Sam3Config.from_pretrained(str(ASSET_DIR), local_files_only=True)
    AutoImageProcessor.from_pretrained(str(processor), local_files_only=True)
    tokenizer = AutoTokenizer.from_pretrained(str(processor), local_files_only=True)
    assert tokenizer('grass')['input_ids'], 'Empty SAM3 tokenizer result'
    with safe_open(model_dir / 'model.safetensors', framework='pt', device='cpu') as tensors:
        prefixes = ('vision_encoder.', 'detector_model.vision_encoder.')
        if not any(key.startswith(prefixes) for key in tensors.keys()):
            raise ValueError('Weights do not contain the SAM3 image vision encoder')


def main():
    """Prepare only the weights kept outside the Navi repository."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model-dir', type=Path, default=checkpoint_directory())
    parser.add_argument('--from-local', type=Path,
                        help='Existing weights or older deployment root')
    parser.add_argument('--check-only', action='store_true',
                        help='Check assets without downloading')
    args = parser.parse_args()
    if args.check_only and args.from_local:
        parser.error('--check-only and --from-local are mutually exclusive')
    model_dir = args.model_dir.expanduser().resolve()
    try:
        if args.check_only:
            validate_assets(model_dir)
        else:
            provenance = (import_local(args.from_local.expanduser().resolve(), model_dir)
                          if args.from_local else download_assets(model_dir))
            validate_assets(model_dir)
            (model_dir / 'checkpoint_source.json').write_text(
                json.dumps(provenance, indent=2) + '\n', encoding='utf-8')
    except Exception as exc:
        from huggingface_hub.errors import GatedRepoError, HfHubHTTPError
        if isinstance(exc, (GatedRepoError, HfHubHTTPError)):
            parser.exit(1, 'SAM3 download failed. Check network/access at '
                        'https://huggingface.co/facebook/sam3 and run '
                        'envs/sam3/.venv/bin/hf auth login from the Navi checkout.\n')
        raise
    print(f'SAM3 checkpoint ready: {model_dir}')
    print(f'Bundled configuration/tokenizer: {ASSET_DIR}')


if __name__ == '__main__':
    main()
