"""Single, explicit import boundary for the bundled hardware SDK."""

from __future__ import annotations

import argparse
import importlib
import sys
from pathlib import Path
from types import ModuleType

from .hardware_config import load_hardware_config
from .paths import WorkspaceLayout


SDK_DIRECTORY = "third_party/reBotArm_control_py"
SDK_PACKAGE = "reBotArm_control_py"


def sdk_root(workspace: WorkspaceLayout | None = None) -> Path:
    """Return the validated bundled SDK directory."""
    layout = workspace or WorkspaceLayout.discover()
    root = layout.resolve(SDK_DIRECTORY)
    if not (root / SDK_PACKAGE / "__init__.py").is_file():
        raise FileNotFoundError(f"incomplete bundled SDK: {SDK_DIRECTORY}")
    return root


def load_sdk(workspace: WorkspaceLayout | None = None) -> ModuleType:
    """Import the bundled SDK without creating or connecting hardware."""
    root = sdk_root(workspace)
    root_text = str(root)
    if root_text not in sys.path:
        # Keep the compatibility workaround in one place. The upstream SDK is
        # not a ROS package yet, so its parent must be importable at runtime.
        sys.path.insert(0, root_text)
    return importlib.import_module(SDK_PACKAGE)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Inspect the bundled SDK without connecting hardware"
    )
    parser.add_argument("--model", default="")
    args = parser.parse_args()

    config = load_hardware_config(model=args.model)
    sdk = load_sdk()
    print(
        f"sdk={sdk.__name__} model={config.model} "
        f"joints={len(config.arm_joints)} connection=not_opened"
    )
