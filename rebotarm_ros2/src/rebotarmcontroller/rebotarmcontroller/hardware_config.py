"""Load and validate hardware configuration without touching hardware."""

from __future__ import annotations

import argparse
import copy
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import yaml

from .paths import WorkspaceLayout


DEFAULT_CONFIG = "src/rebotarm_bringup/config/rebotarm_hardware.yaml"


@dataclass(frozen=True)
class HardwareConfig:
    """One fully merged and validated robot model configuration."""

    model: str
    source: Path
    sdk_source: Path
    data: dict[str, Any]

    @property
    def arm_joints(self) -> tuple[str, ...]:
        return tuple(self.data["groups"]["arm"]["joints"])

    @property
    def gripper_joints(self) -> tuple[str, ...]:
        group = self.data.get("groups", {}).get("gripper", {})
        return tuple(group.get("joints", []))


def load_hardware_config(
    config: str | Path = DEFAULT_CONFIG,
    *,
    model: str = "",
    channel: str = "",
    workspace: WorkspaceLayout | None = None,
) -> HardwareConfig:
    """Load SDK defaults, apply ROS overrides, and validate the result."""
    layout = workspace or WorkspaceLayout.discover()
    config_path = layout.resolve(config)
    root_config = _read_mapping(config_path, "hardware config")

    selected = str(model or root_config.get("default_model", "")).strip().lower()
    models = _mapping(root_config.get("models"), "models")
    if not selected:
        raise ValueError("default_model or model argument is required")
    if selected not in models:
        choices = ", ".join(sorted(str(name) for name in models))
        raise ValueError(f"unknown hardware model {selected!r}; choices: {choices}")

    model_config = _mapping(models[selected], f"models.{selected}")
    sdk_relative = model_config.get("sdk_config")
    if not isinstance(sdk_relative, str) or not sdk_relative.strip():
        raise ValueError(f"models.{selected}.sdk_config must be a relative path")
    sdk_path = layout.resolve(sdk_relative)
    merged = _deep_merge(
        _read_mapping(sdk_path, "SDK config"),
        _mapping(model_config.get("overrides", {}), f"models.{selected}.overrides"),
    )
    if channel:
        merged["channel"] = channel

    _validate(merged)
    return HardwareConfig(selected, config_path, sdk_path, merged)


def _read_mapping(path: Path, label: str) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    return dict(_mapping(value, label))


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{label} must be a YAML mapping")
    return value


def _deep_merge(base: Any, override: Any) -> Any:
    if isinstance(base, Mapping) and isinstance(override, Mapping):
        result = copy.deepcopy(dict(base))
        for key, value in override.items():
            result[key] = _deep_merge(result.get(key), value)
        return result
    return copy.deepcopy(override)


def _validate(data: Mapping[str, Any]) -> None:
    channel = data.get("channel")
    if not isinstance(channel, str) or not channel.strip():
        raise ValueError("channel must be a non-empty string")

    rate = data.get("rate")
    if not isinstance(rate, (int, float)) or isinstance(rate, bool) or rate <= 0:
        raise ValueError("rate must be greater than zero")

    joints = data.get("joints")
    if not isinstance(joints, list) or not joints:
        raise ValueError("joints must be a non-empty list")
    names = []
    for index, joint in enumerate(joints):
        item = _mapping(joint, f"joints[{index}]")
        name = item.get("name")
        if not isinstance(name, str) or not name.strip():
            raise ValueError(f"joints[{index}].name must be a non-empty string")
        names.append(name)
    if len(names) != len(set(names)):
        raise ValueError("joint names must be unique")

    groups = _mapping(data.get("groups"), "groups")
    arm = _mapping(groups.get("arm"), "groups.arm")
    arm_joints = arm.get("joints")
    if not isinstance(arm_joints, list) or not arm_joints:
        raise ValueError("groups.arm.joints must be a non-empty list")
    unknown = [name for name in arm_joints if name not in names]
    if unknown:
        raise ValueError(f"groups.arm contains unknown joints: {unknown}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Validate reBotArm hardware YAML")
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--model", default="")
    parser.add_argument("--channel", default="")
    args = parser.parse_args()
    loaded = load_hardware_config(args.config, model=args.model, channel=args.channel)
    print(
        f"valid model={loaded.model} channel={loaded.data['channel']} "
        f"rate={loaded.data['rate']} arm_joints={len(loaded.arm_joints)}"
    )
