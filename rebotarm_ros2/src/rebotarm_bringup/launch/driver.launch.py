from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params_file = PathJoinSubstitution(
        [FindPackageShare("rebotarm_bringup"), "config", "driver_params.yaml"]
    )
    use_real_hardware = LaunchConfiguration("use_real_hardware")
    channel = LaunchConfiguration("channel")
    publish_rate = LaunchConfiguration("publish_rate")
    use_rviz = LaunchConfiguration("use_rviz")
    allow_enable = LaunchConfiguration("allow_enable")
    allow_safe_home = LaunchConfiguration("allow_safe_home")
    allow_joint_motion = LaunchConfiguration("allow_joint_motion")
    allow_gravity_compensation = LaunchConfiguration("allow_gravity_compensation")
    allow_calibration = LaunchConfiguration("allow_calibration")
    calibration_mode = LaunchConfiguration("calibration_mode")
    urdf_file = PathJoinSubstitution(
        [
            FindPackageShare("rebotarm_bringup"),
            "description",
            "urdf",
            "reBot-DevArm_fixend.urdf",
        ]
    )
    rviz_file = PathJoinSubstitution(
        [FindPackageShare("rebotarm_bringup"), "rviz", "rebotarm.rviz"]
    )
    robot_description = ParameterValue(Command(["xacro ", urdf_file]), value_type=str)

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_real_hardware",
                default_value="false",
                description="Explicitly connect the DM arm through SocketCAN",
            ),
            DeclareLaunchArgument(
                "channel",
                default_value="can0",
                description="SocketCAN interface used by DM-USB2FDCAN",
            ),
            DeclareLaunchArgument(
                "publish_rate",
                default_value="20.0",
                description="Joint-state and diagnostic publication rate in Hz",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description="Start RViz with the DM arm model",
            ),
            DeclareLaunchArgument(
                "allow_enable",
                default_value="false",
                description="Unlock the state-changing /rebotarm/enable service",
            ),
            DeclareLaunchArgument(
                "allow_safe_home",
                default_value="false",
                description="Unlock calibrated-zero minimum-jerk motion",
            ),
            DeclareLaunchArgument(
                "allow_joint_motion",
                default_value="false",
                description="Unlock bounded FollowJointTrajectory motion",
            ),
            DeclareLaunchArgument(
                "allow_gravity_compensation",
                default_value="false",
                description="Unlock low-gain model gravity compensation",
            ),
            DeclareLaunchArgument(
                "allow_calibration",
                default_value="false",
                description="Unlock maintenance-only calibration APIs",
            ),
            DeclareLaunchArgument(
                "calibration_mode",
                default_value="false",
                description="Confirm the arm is mechanically positioned for calibration",
            ),
            Node(
                package="rebotarmcontroller",
                executable="joint_state_node",
                name="rebotarm_joint_state",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "use_real_hardware": ParameterValue(
                            use_real_hardware, value_type=bool
                        ),
                        "channel": channel,
                        "publish_rate": ParameterValue(publish_rate, value_type=float),
                        "allow_enable": ParameterValue(allow_enable, value_type=bool),
                        "allow_safe_home": ParameterValue(
                            allow_safe_home, value_type=bool
                        ),
                        "allow_joint_motion": ParameterValue(
                            allow_joint_motion, value_type=bool
                        ),
                        "allow_gravity_compensation": ParameterValue(
                            allow_gravity_compensation, value_type=bool
                        ),
                        "allow_calibration": ParameterValue(
                            allow_calibration, value_type=bool
                        ),
                        "calibration_mode": ParameterValue(
                            calibration_mode, value_type=bool
                        ),
                    },
                ],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_file],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
