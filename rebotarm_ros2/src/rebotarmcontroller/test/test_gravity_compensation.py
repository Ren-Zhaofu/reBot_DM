import time

from rebotarmcontroller.fake_robot import fake_robot_factory
from rebotarmcontroller.hardware_config import load_hardware_config
from rebotarmcontroller.hardware_manager import HardwareManager


def test_gravity_compensation_fake_lifecycle() -> None:
    config = load_hardware_config(model="dm")
    manager = HardwareManager(config, robot_factory=fake_robot_factory)
    manager.connect()
    manager.start_gravity_compensation()
    time.sleep(0.05)
    active = manager.diagnostics()
    assert active.mode == "gravity_compensation"
    assert active.control_loop_active is True
    assert active.enabled is True
    assert active.state_machine == "GRAVITY_COMPENSATION"

    manager.stop_gravity_compensation()
    stopped = manager.diagnostics()
    assert stopped.mode == "position_hold"
    assert stopped.control_loop_active is False
    assert stopped.enabled is True
    manager.disable()
    assert manager.diagnostics().enabled is False
    manager.close()
