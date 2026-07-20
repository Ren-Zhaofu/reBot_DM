"""Relocatable workspace path handling.

Configuration paths are workspace-relative. They never depend on the shell's
current working directory and may not escape the workspace.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


_WORKSPACE_MARKERS = ("src", "third_party")


def find_workspace_root(start: str | Path | None = None) -> Path:
    """Find the containing rebotarm_ros2 workspace from a source/install path."""
    origin = Path(start or __file__).expanduser().resolve()
    current = origin if origin.is_dir() else origin.parent

    for candidate in (current, *current.parents):
        if all((candidate / marker).is_dir() for marker in _WORKSPACE_MARKERS):
            return candidate

    raise FileNotFoundError(
        f"cannot locate the rebotarm_ros2 workspace from {origin}; "
        "expected sibling src/ and third_party/ directories"
    )


@dataclass(frozen=True)
class WorkspaceLayout:
    """Resolve and validate paths owned by one workspace."""

    root: Path

    @classmethod
    def discover(cls, start: str | Path | None = None) -> "WorkspaceLayout":
        return cls(find_workspace_root(start))

    def resolve(self, relative_path: str | Path, *, must_exist: bool = True) -> Path:
        value = Path(relative_path)
        if value.is_absolute():
            raise ValueError(f"absolute paths are forbidden: {value}")

        resolved = (self.root / value).resolve()
        try:
            resolved.relative_to(self.root)
        except ValueError as exc:
            raise ValueError(f"path escapes the workspace: {value}") from exc

        if must_exist and not resolved.exists():
            raise FileNotFoundError(f"workspace resource not found: {value}")
        return resolved

