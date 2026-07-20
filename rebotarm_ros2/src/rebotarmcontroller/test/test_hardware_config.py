from pathlib import Path

import pytest

from rebotarmcontroller.hardware_config import load_hardware_config
from rebotarmcontroller.paths import WorkspaceLayout


SDK = """
channel: original
rate: 500
groups:
  arm:
    joints: [joint1]
joints:
  - name: joint1
    motor_id: 1
"""

SELECTOR = """
default_model: dm
models:
  dm:
    sdk_config: third_party/sdk/config/dm.yaml
    overrides:
      rate: 100
"""


def workspace(tmp_path: Path) -> WorkspaceLayout:
    root = tmp_path / "rebotarm_ros2"
    (root / "src" / "rebotarm_bringup" / "config").mkdir(parents=True)
    (root / "third_party" / "sdk" / "config").mkdir(parents=True)
    (root / "third_party" / "sdk" / "config" / "dm.yaml").write_text(SDK)
    (root / "src" / "rebotarm_bringup" / "config" / "hardware.yaml").write_text(
        SELECTOR
    )
    return WorkspaceLayout(root)


def test_loads_merges_and_overrides_channel(tmp_path: Path, monkeypatch) -> None:
    layout = workspace(tmp_path)
    monkeypatch.chdir(tmp_path)
    loaded = load_hardware_config(
        "src/rebotarm_bringup/config/hardware.yaml",
        channel="can9",
        workspace=layout,
    )
    assert loaded.model == "dm"
    assert loaded.data["rate"] == 100
    assert loaded.data["channel"] == "can9"
    assert loaded.arm_joints == ("joint1",)


def test_rejects_unknown_model(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="unknown hardware model"):
        load_hardware_config(
            "src/rebotarm_bringup/config/hardware.yaml",
            model="missing",
            workspace=workspace(tmp_path),
        )


def test_rejects_absolute_sdk_path(tmp_path: Path) -> None:
    layout = workspace(tmp_path)
    selector = layout.root / "src/rebotarm_bringup/config/hardware.yaml"
    selector.write_text(SELECTOR.replace("third_party/sdk/config/dm.yaml", "/tmp/dm.yaml"))
    with pytest.raises(ValueError, match="absolute paths are forbidden"):
        load_hardware_config(
            "src/rebotarm_bringup/config/hardware.yaml", workspace=layout
        )


def test_rejects_unknown_group_joint(tmp_path: Path) -> None:
    layout = workspace(tmp_path)
    sdk = layout.root / "third_party/sdk/config/dm.yaml"
    sdk.write_text(SDK.replace("[joint1]", "[missing]"))
    with pytest.raises(ValueError, match="unknown joints"):
        load_hardware_config(
            "src/rebotarm_bringup/config/hardware.yaml", workspace=layout
        )
