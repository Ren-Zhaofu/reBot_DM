import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_real_hardware", default_value="false"),
            DeclareLaunchArgument("channel", default_value="can0"),
            DeclareLaunchArgument("allow_joint_motion", default_value="false"),
            DeclareLaunchArgument("allow_safe_home", default_value="false"),
            DeclareLaunchArgument("allow_cartesian_motion", default_value="false"),
            OpaqueFunction(function=_launch_setup),
        ]
    )


def _launch_setup(context):
    config = (
        MoveItConfigsBuilder("reBot-DevArm_fixend", package_name="rebotarm_moveit_config")
        .robot_description(file_path="config/rebotarm.urdf.xacro")
        .robot_description_semantic(file_path="config/rebotarm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )
    parameters = config.to_dict()
    if os.environ.get("ROS_DISTRO") == "humble":
        ompl = parameters.setdefault("ompl", {})
        ompl["planning_plugin"] = "ompl_interface/OMPLPlanner"
        ompl["request_adapters"] = " ".join(
            [
                "default_planner_request_adapters/AddTimeOptimalParameterization",
                "default_planner_request_adapters/ResolveConstraintFrames",
                "default_planner_request_adapters/FixWorkspaceBounds",
                "default_planner_request_adapters/FixStartStateBounds",
                "default_planner_request_adapters/FixStartStateCollision",
            ]
        )
        ompl.pop("response_adapters", None)

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("rebotarm_bringup"), "launch", "driver.launch.py"]
            )
        ),
        launch_arguments={
            "use_real_hardware": LaunchConfiguration("use_real_hardware"),
            "channel": LaunchConfiguration("channel"),
            "allow_joint_motion": LaunchConfiguration("allow_joint_motion"),
            "allow_safe_home": LaunchConfiguration("allow_safe_home"),
            "use_rviz": "false",
        }.items(),
    )
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[parameters],
    )
    cartesian_bridge = Node(
        package="rebotarmcontroller",
        executable="cartesian_bridge",
        name="rebotarm_cartesian_bridge",
        output="screen",
        parameters=[
            {
                "allow_cartesian_motion": LaunchConfiguration(
                    "allow_cartesian_motion"
                )
            }
        ],
    )
    return [driver, move_group, cartesian_bridge]
