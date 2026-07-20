from dataclasses import dataclass

import pytest

from rebotarmcontroller.read_only_check import run_read_only_check, validate_channel_ready
from rebotarmcontroller.state import JointStateSnapshot


@dataclass
class Config:
    model: str = "fake"
    data = {"channel": "fake0"}
    arm_joints = ("joint1",)


class FakeManager:
    def __init__(self, *, read_error: Exception | None = None) -> None:
        self.config = Config()
        self.connect_calls = 0
        self.close_calls = 0
        self.read_calls = 0
        self.read_error = read_error

    def connect(self) -> None:
        self.connect_calls += 1

    def read_state(self) -> JointStateSnapshot:
        self.read_calls += 1
        if self.read_error is not None:
            raise self.read_error
        return JointStateSnapshot.from_sdk(
            names=("joint1",),
            position=(1.25,),
            velocity=(0.0,),
            effort=(0.0,),
            monotonic_ns=1,
        )

    def close(self) -> None:
        self.close_calls += 1


def test_dry_run_never_connects_or_closes() -> None:
    manager = FakeManager()
    output = []
    result = run_read_only_check(
        manager, connect=False, samples=1, interval=0.1, emit=output.append
    )
    assert result == 0
    assert manager.connect_calls == 0
    assert manager.read_calls == 0
    assert manager.close_calls == 0
    assert "not opened" in output[-1]


def test_connected_run_is_bounded_and_always_closes() -> None:
    manager = FakeManager()
    sleeps = []
    result = run_read_only_check(
        manager,
        connect=True,
        samples=3,
        interval=0.1,
        emit=lambda _: None,
        sleep=sleeps.append,
    )
    assert result == 0
    assert manager.connect_calls == 1
    assert manager.read_calls == 3
    assert manager.close_calls == 1
    assert sleeps == [0.1, 0.1]


def test_read_failure_still_closes() -> None:
    manager = FakeManager(read_error=OSError("feedback failed"))
    with pytest.raises(OSError, match="feedback failed"):
        run_read_only_check(
            manager,
            connect=True,
            samples=2,
            interval=0.1,
            emit=lambda _: None,
        )
    assert manager.close_calls == 1


def test_keyboard_interrupt_still_closes() -> None:
    manager = FakeManager(read_error=KeyboardInterrupt())
    output = []
    result = run_read_only_check(
        manager,
        connect=True,
        samples=2,
        interval=0.1,
        emit=output.append,
    )
    assert result == 130
    assert manager.close_calls == 1
    assert "interrupted: disconnecting" in output
    assert output[-1] == "disconnected"


@pytest.mark.parametrize(
    ("samples", "interval", "message"),
    [(0, 0.1, "samples"), (101, 0.1, "samples"), (1, 0.0, "interval")],
)
def test_rejects_unbounded_or_fast_checks(samples, interval, message) -> None:
    with pytest.raises(ValueError, match=message):
        run_read_only_check(
            FakeManager(),
            connect=True,
            samples=samples,
            interval=interval,
            emit=lambda _: None,
        )


def test_preflight_rejects_down_socketcan(tmp_path, monkeypatch) -> None:
    manager = FakeManager()
    manager.config.data = {"channel": "can0", "transport": "socketcan"}
    real_open = open

    def fake_open(path, *args, **kwargs):
        if path == "/sys/class/net/can0/flags":
            flags = tmp_path / "flags"
            flags.write_text("0x0")
            return real_open(flags, *args, **kwargs)
        return real_open(path, *args, **kwargs)

    monkeypatch.setattr("builtins.open", fake_open)
    with pytest.raises(RuntimeError, match="can0 is DOWN"):
        validate_channel_ready(manager)
