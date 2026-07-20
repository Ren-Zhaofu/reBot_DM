"""Bounded, opt-in real-hardware read-only smoke test."""

from __future__ import annotations

import argparse
import os
import time
from collections.abc import Callable

from .hardware_config import load_hardware_config
from .hardware_manager import HardwareManager


MAX_SAMPLES = 100
MIN_INTERVAL_SECONDS = 0.02


def validate_channel_ready(manager: HardwareManager) -> None:
    """Fail before SDK access when the configured OS device is unavailable."""
    channel = str(manager.config.data["channel"])
    transport = str(manager.config.data.get("transport", "")).lower()
    if transport in ("socketcan", "socketcanfd"):
        flags_path = f"/sys/class/net/{channel}/flags"
        try:
            flags = int(open(flags_path, encoding="ascii").read().strip(), 16)
        except FileNotFoundError as exc:
            raise RuntimeError(f"SocketCAN interface does not exist: {channel}") from exc
        if flags & 0x1 == 0:  # Linux IFF_UP
            raise RuntimeError(
                f"SocketCAN interface {channel} is DOWN; configure its bitrate and bring it UP"
            )
        return

    if channel.startswith("/dev/"):
        if not os.path.exists(channel):
            raise RuntimeError(f"serial device does not exist: {channel}")
        if not os.access(channel, os.R_OK | os.W_OK):
            raise RuntimeError(f"serial device is not readable and writable: {channel}")


def run_read_only_check(
    manager: HardwareManager,
    *,
    connect: bool,
    samples: int,
    interval: float,
    emit: Callable[[str], None] = print,
    sleep: Callable[[float], None] = time.sleep,
) -> int:
    """Read a bounded number of samples and always close the manager."""
    if not 1 <= samples <= MAX_SAMPLES:
        raise ValueError(f"samples must be between 1 and {MAX_SAMPLES}")
    if interval < MIN_INTERVAL_SECONDS:
        raise ValueError(f"interval must be at least {MIN_INTERVAL_SECONDS} seconds")

    emit(
        f"model={manager.config.model} channel={manager.config.data['channel']} "
        f"joints={len(manager.config.arm_joints)}"
    )
    if not connect:
        emit("dry-run: hardware connection not opened; pass --connect to opt in")
        return 0

    emit(
        "SAFETY: read-only feedback; no enable or motion command will be sent; "
        "SDK disconnect will disable all configured motors"
    )
    try:
        manager.connect()
        emit("connected")
        for index in range(samples):
            state = manager.read_state()
            positions = ", ".join(f"{value:.6f}" for value in state.position)
            emit(f"sample={index + 1}/{samples} position=[{positions}]")
            if index + 1 < samples:
                sleep(interval)
    except KeyboardInterrupt:
        emit("interrupted: disconnecting")
        return 130
    finally:
        manager.close()
        emit("disconnected")
    return 0


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Bounded reBotArm feedback check; dry-run unless --connect is set"
    )
    parser.add_argument("--model", choices=("dm", "rs"), default="dm")
    parser.add_argument("--channel", default="")
    parser.add_argument("--samples", type=int, default=10)
    parser.add_argument("--interval", type=float, default=0.1)
    parser.add_argument(
        "--connect",
        action="store_true",
        help="explicitly allow communication with the configured hardware",
    )
    return parser


def main() -> int:
    args = _parser().parse_args()
    config = load_hardware_config(model=args.model, channel=args.channel)
    manager = HardwareManager(config)
    if args.connect:
        validate_channel_ready(manager)
    return run_read_only_check(
        manager,
        connect=args.connect,
        samples=args.samples,
        interval=args.interval,
    )
