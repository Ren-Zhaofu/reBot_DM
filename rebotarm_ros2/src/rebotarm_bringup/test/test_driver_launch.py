import importlib.util
from pathlib import Path

from launch import LaunchDescription


def test_driver_launch_generates_description() -> None:
    launch_file = Path(__file__).parents[1] / "launch" / "driver.launch.py"
    spec = importlib.util.spec_from_file_location("driver_launch", launch_file)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    assert isinstance(module.generate_launch_description(), LaunchDescription)

