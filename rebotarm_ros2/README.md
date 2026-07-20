# reBotArm ROS 2 (rebuild)

This workspace is self-contained and relocatable. Move the entire
`rebotarm_ros2` directory, rebuild it, and source the new install space.

The rebuilt hardware target is the DM-motor arm using a DM-USB2FDCAN
SocketCAN adapter. RS hardware is not part of the active migration target.

```bash
colcon build --symlink-install
source install/setup.bash
```

Project files must not contain machine-specific absolute paths. Paths stored in
configuration are relative to the workspace root and are resolved independently
of the process working directory.

## Current rebuild status

- Portable workspace path discovery
- Offline DM/RS configuration loading and validation
- Bundled SDK import boundary (`inspect_sdk`), with no automatic hardware connection
- Explicit `HardwareManager` lifecycle with idempotent connect/disconnect handling
- Immutable, validated arm joint-state snapshots
- Bounded, explicit opt-in read-only hardware smoke-test command
- Standard `/joint_states` publisher with fake-by-default and explicit real-DM mode
- Latched `/rebotarm/arm_status` diagnostics with per-joint DM status codes
- Six `/rebotarm/joints/jointN/state` `JointMotorState` topics
- Complete DM URDF/STL model, TF publication, and optional RViz visualization
- `/rebotarm/disable` service with propagated DM controller failures
- Locked-by-default `/rebotarm/enable` service (`allow_enable:=true` required)
- Complete-feedback wait on connect, preventing partial zero-filled startup samples
- Current-position POS_VEL hold priming with automatic 0.05 rad error disable
- Bounded 50 Hz minimum-jerk safe-home trajectory planner
- Locked `/rebotarm/safe_home` service targeting the existing calibrated zero
- Double-locked, single-joint maintenance `/rebotarm/set_zero` API
- Locked-by-default `/rebotarm/follow_joint_trajectory` standard action
- Bounded minimum-jerk joint motion with limits, cancellation, settling, and fault disable
- Complete desired/actual/error/time action feedback and MoveIt tolerance handling
- Arm-only MoveIt 2 configuration connected directly to the portable driver action

Real-DM safe-home reached a maximum zero error of `0.002480 rad` directly and
about `0.000572 rad` after settling; the ROS service path was also verified.

The real-DM POS_VEL hold test observed `0.000000 rad` maximum movement across
20 samples over one second, then disabled and disconnected cleanly.

The bounded real-DM enable test observed `0.000000 rad` maximum joint movement
before disabling again.
- Real-DM `joint6` motion reached `0.020409 rad` for a `0.020000 rad` target,
  returned to `0.000191 rad`, and then disabled with zero CAN errors.
- Safe-by-default `rebotarm_bringup/driver.launch.py` for fake or explicit real-DM use

The real DM + DM-USB2FDCAN path has been verified on `can0` at 1 Mbps with no
reported CAN errors or dropped frames during the initial read-only test.

```bash
# Safe simulated feedback (default)
ros2 launch rebotarm_bringup driver.launch.py

# Real DM feedback through DM-USB2FDCAN
ros2 launch rebotarm_bringup driver.launch.py \
  use_real_hardware:=true channel:=can0 publish_rate:=10.0 use_rviz:=true

# MoveIt planning with safe fake feedback; execution remains locked
ros2 launch rebotarm_moveit_config moveit.launch.py

# Explicit real hardware execution (only after the arm is made safe)
ros2 launch rebotarm_moveit_config moveit.launch.py \
  use_real_hardware:=true channel:=can0 allow_joint_motion:=true
```

The rebuilt controller does not connect unless `HardwareManager.connect()` is
called explicitly. All state-changing APIs remain locked unless their launch
arguments are explicitly enabled.
