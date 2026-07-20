from pathlib import Path

import pytest

from rebotarmcontroller.paths import WorkspaceLayout, find_workspace_root


def make_workspace(tmp_path: Path) -> Path:
    root = tmp_path / "moved_anywhere" / "rebotarm_ros2"
    (root / "src" / "package").mkdir(parents=True)
    (root / "third_party" / "sdk" / "config").mkdir(parents=True)
    (root / "third_party" / "sdk" / "config" / "hardware.yaml").touch()
    return root


def test_discovers_workspace_after_move(tmp_path: Path) -> None:
    root = make_workspace(tmp_path)
    assert find_workspace_root(root / "src" / "package") == root


def test_resolves_from_workspace_not_cwd(tmp_path: Path, monkeypatch) -> None:
    root = make_workspace(tmp_path)
    monkeypatch.chdir(tmp_path)
    layout = WorkspaceLayout.discover(root / "src" / "package")
    assert layout.resolve("third_party/sdk/config/hardware.yaml") == (
        root / "third_party" / "sdk" / "config" / "hardware.yaml"
    )


def test_rejects_absolute_path(tmp_path: Path) -> None:
    layout = WorkspaceLayout(make_workspace(tmp_path))
    with pytest.raises(ValueError, match="absolute paths are forbidden"):
        layout.resolve(tmp_path / "hardware.yaml")


def test_rejects_workspace_escape(tmp_path: Path) -> None:
    layout = WorkspaceLayout(make_workspace(tmp_path))
    with pytest.raises(ValueError, match="escapes the workspace"):
        layout.resolve("../outside.yaml", must_exist=False)

