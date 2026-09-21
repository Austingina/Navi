"""Check repository-local checkpoint imports and non-weight assets shipped with Navi."""

import importlib.util
import json
from pathlib import Path

import pytest


spec = importlib.util.spec_from_file_location(
    'prepare_assets', Path(__file__).resolve().parents[1] / 'prepare_assets.py')
assets = importlib.util.module_from_spec(spec)
spec.loader.exec_module(assets)


@pytest.fixture
def bundle(tmp_path):
    root = tmp_path / 'old-machine'
    model = root / 'checkpoints/sam3'
    model.mkdir(parents=True)
    (model / 'model.safetensors').write_bytes(b'test weights')
    (model / 'config.json').write_text('Do not copy external configuration')
    (root / '.venv').mkdir()
    return root


@pytest.mark.parametrize('source_kind', ['root', 'directory', 'file'])
def test_import_only_checkpoint(bundle, tmp_path, source_kind):
    target = tmp_path / 'weights'
    source = {'root': bundle, 'directory': bundle / 'checkpoints/sam3',
              'file': bundle / 'checkpoints/sam3/model.safetensors'}[source_kind]
    provenance = assets.import_local(source, target)
    assert provenance['source'] == 'local'
    assert (target / 'model.safetensors').read_bytes() == b'test weights'
    assert sorted(p.name for p in target.iterdir()) == ['model.safetensors']


def test_missing_weight_keeps_existing_install(tmp_path):
    target = tmp_path / 'weights'
    target.mkdir()
    (target / 'model.safetensors').write_bytes(b'working weights')
    with pytest.raises(FileNotFoundError, match='No model.safetensors'):
        assets.import_local(tmp_path / 'missing', target)
    assert (target / 'model.safetensors').read_bytes() == b'working weights'


def test_import_same_directory_keeps_checkpoint(bundle):
    directory = bundle / 'checkpoints/sam3'
    assets.import_local(directory, directory)
    assert (directory / 'model.safetensors').read_bytes() == b'test weights'


def test_native_pt_is_not_accepted_as_hf_weights(tmp_path):
    checkpoint = tmp_path / 'sam3.pt'
    checkpoint.write_bytes(b'native weights')
    with pytest.raises(ValueError, match='HF .safetensors'):
        assets.import_local(checkpoint, tmp_path / 'target')
    assert not (tmp_path / 'target').exists()


def test_clone_contains_all_non_weight_assets():
    for filename in ('config.json', 'LICENSE', 'processor/preprocessor_config.json',
                     'processor/tokenizer.json', 'processor/tokenizer_config.json'):
        path = assets.ASSET_DIR / filename
        assert path.is_file() and not path.is_symlink()
        if path.suffix == '.json':
            json.loads(path.read_text())
    assert not list(assets.ASSET_DIR.rglob('*.safetensors'))
