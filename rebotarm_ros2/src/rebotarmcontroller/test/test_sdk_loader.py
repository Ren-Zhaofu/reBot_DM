import sys

from rebotarmcontroller.paths import WorkspaceLayout
from rebotarmcontroller.sdk_loader import load_sdk, sdk_root


def test_bundled_sdk_is_complete() -> None:
    root = sdk_root()
    assert (root / "reBotArm_control_py/actuator/rebotarm.py").is_file()
    assert (root / "reBotArm_control_py/controllers").is_dir()


def test_loading_sdk_does_not_create_hardware(monkeypatch) -> None:
    # Importing must not invoke the SDK's explicit connect method.
    module = load_sdk()
    rebotarm_type = module.actuator.RebotArm
    called = False

    def forbidden_connect(self):
        nonlocal called
        called = True
        raise AssertionError("connect must not run during SDK import")

    monkeypatch.setattr(rebotarm_type, "connect", forbidden_connect)
    assert load_sdk() is module
    assert called is False


def test_loader_registers_only_the_discovered_sdk_root() -> None:
    layout = WorkspaceLayout.discover()
    load_sdk(layout)
    assert str(sdk_root(layout)) in sys.path
