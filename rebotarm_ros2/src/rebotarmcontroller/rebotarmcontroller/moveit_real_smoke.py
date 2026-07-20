"""Guarded MoveIt real-arm smoke test used by the shell entry point."""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger


JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))


class MoveItRealSmoke(Node):
    def __init__(self) -> None:
        super().__init__("rebotarm_moveit_real_smoke")
        self.home = self.create_client(Trigger, "/rebotarm/safe_home")
        self.disable = self.create_client(Trigger, "/rebotarm/disable")
        self.move_group = ActionClient(self, MoveGroup, "/move_action")

    def wait_until_ready(self) -> None:
        if not self.home.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("/rebotarm/safe_home 服务不可用")
        if not self.disable.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("/rebotarm/disable 服务不可用")
        if not self.move_group.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("/move_action 不可用")

    def trigger(self, client, label: str, timeout: float) -> None:
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        response = future.result()
        if response is None or not response.success:
            detail = response.message if response is not None else "超时"
            raise RuntimeError(f"{label}失败: {detail}")
        print(f"{label}: {response.message}")

    def move_to(self, positions: tuple[float, ...], label: str) -> None:
        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.num_planning_attempts = 3
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.2
        goal.request.max_acceleration_scaling_factor = 0.2
        goal.request.start_state.is_diff = True
        constraint = Constraints(name=label)
        for name, position in zip(JOINT_NAMES, positions):
            constraint.joint_constraints.append(
                JointConstraint(
                    joint_name=name,
                    position=position,
                    tolerance_above=0.002,
                    tolerance_below=0.002,
                    weight=1.0,
                )
            )
        goal.request.goal_constraints = [constraint]
        send = self.move_group.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=10.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f"{label}: MoveIt 拒绝目标")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        wrapped = result_future.result()
        if wrapped is None:
            handle.cancel_goal_async()
            raise RuntimeError(f"{label}: MoveIt 执行超时")
        planned = wrapped.result.planned_trajectory.joint_trajectory
        print(f"{label}: MoveIt 生成 {len(planned.points)} 个轨迹点")
        if planned.points:
            endpoint = [round(value, 6) for value in planned.points[-1].positions]
            print(f"{label}: 规划末点={endpoint}")
        if wrapped.result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                f"{label}: MoveIt error_code={wrapped.result.error_code.val}"
            )
        print(f"{label}: MoveIt 规划与执行成功")


def _confirmed(assume_yes: bool) -> bool:
    if assume_yes:
        return True
    if not sys.stdin.isatty():
        print("非交互执行必须传入 --yes", file=sys.stderr)
        return False
    print("警告：将真机回零，并让 joint6 在 0 与 +0.02 rad 之间运动。")
    return input("确认周围安全且急停可用？输入 yes 继续: ").strip() == "yes"


def main() -> int:
    parser = argparse.ArgumentParser(description="MoveIt real DM joint6 smoke test")
    parser.add_argument("--yes", action="store_true", help="skip interactive prompt")
    args = parser.parse_args()
    if not _confirmed(args.yes):
        print("已取消。", file=sys.stderr)
        return 2

    rclpy.init()
    node = MoveItRealSmoke()
    failed = False
    try:
        node.wait_until_ready()
        node.trigger(node.home, "安全回零", 30.0)
        time.sleep(1.0)
        node.move_to((0.0, 0.0, 0.0, 0.0, 0.0, 0.02), "joint6 +0.02 rad")
        time.sleep(0.5)
        node.move_to((0.0,) * 6, "返回标定零位")
        print("MoveIt 真机小幅闭环验证通过。")
    except (KeyboardInterrupt, Exception) as exc:
        failed = True
        print(f"验证中止: {exc}", file=sys.stderr)
    finally:
        try:
            node.trigger(node.disable, "最终失能", 5.0)
        except Exception as exc:
            failed = True
            print(f"严重警告：自动失能失败: {exc}", file=sys.stderr)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
