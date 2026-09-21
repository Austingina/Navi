"""Resolve in-repository SAM3 runtime and checkpoint locations."""

import os
from pathlib import Path


def repository_root():
    """Find the Navi checkout from an explicit setting or this source/install path."""
    for name in ('NAVI_ROOT', 'NAVI_ROOT_DIR'):
        if os.environ.get(name):
            return Path(os.environ[name]).expanduser()
    for root in Path(__file__).resolve().parents:
        if (root / 'envs/sam3/install.sh').is_file():
            return root
    raise RuntimeError('Cannot locate Navi repository; set NAVI_ROOT or NAVI_ROOT_DIR')


def checkpoint_directory():
    """Return the ignored repository-local checkpoint directory."""
    override = os.environ.get('SAM3_CHECKPOINT_DIR')
    return Path(override).expanduser() if override else repository_root() / 'checkpoints/sam3'


def runtime_python(package_share):
    """Find envs/sam3/.venv from a source, isolated install or merged install layout."""
    roots = [Path(os.environ[name]).expanduser() for name in ('NAVI_ROOT', 'NAVI_ROOT_DIR')
             if os.environ.get(name)]
    roots.extend(Path(package_share).resolve().parents)
    roots.extend(Path(__file__).resolve().parents)
    for root in roots:
        if (root / 'envs/sam3/install.sh').is_file():
            return root / 'envs/sam3/.venv/bin/python'
    raise RuntimeError(
        'Cannot locate Navi/envs/sam3. Set NAVI_ROOT to the repository root '
        'or pass sam3_python explicitly.')
