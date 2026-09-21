"""Locate the repository runtime after cloning under a different directory."""

from pathlib import Path

import pytest

from navi_semantic_perception.sam3_paths import checkpoint_directory, runtime_python


@pytest.mark.parametrize('layout', [
    'ros2_ws/src/navi_semantic_perception',
    'ros2_ws/install/navi_semantic_perception/share/navi_semantic_perception',
    'ros2_ws/install/share/navi_semantic_perception',
])
def test_runtime_follows_clone_location(tmp_path, monkeypatch, layout):
    monkeypatch.delenv('NAVI_ROOT', raising=False)
    monkeypatch.delenv('NAVI_ROOT_DIR', raising=False)
    repo = tmp_path / 'another clone'
    setup = repo / 'envs/sam3/install.sh'
    setup.parent.mkdir(parents=True)
    setup.touch()
    share = repo / layout
    share.mkdir(parents=True)
    assert runtime_python(share) == repo / 'envs/sam3/.venv/bin/python'


def test_explicit_repo_root_supports_detached_ros_install(tmp_path, monkeypatch):
    setup = tmp_path / 'envs/sam3/install.sh'
    setup.parent.mkdir(parents=True)
    setup.touch()
    monkeypatch.setenv('NAVI_ROOT', str(tmp_path))
    assert runtime_python('/opt/ros/navi/share/navi_semantic_perception') == (
        tmp_path / 'envs/sam3/.venv/bin/python')


def test_default_checkpoint_is_in_repository(monkeypatch):
    monkeypatch.delenv('SAM3_CHECKPOINT_DIR', raising=False)
    monkeypatch.setenv('NAVI_ROOT', '/tmp/navi-checkout')
    assert checkpoint_directory() == Path('/tmp/navi-checkout/checkpoints/sam3')


def test_checkpoint_override(monkeypatch, tmp_path):
    monkeypatch.setenv('SAM3_CHECKPOINT_DIR', str(tmp_path))
    assert checkpoint_directory() == tmp_path
