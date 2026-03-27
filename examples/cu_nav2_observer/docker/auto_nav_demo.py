#!/usr/bin/env python3

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def make_pose(navigator: BasicNavigator, x: float, y: float, yaw_rad: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw_rad * 0.5)
    pose.pose.orientation.w = math.cos(yaw_rad * 0.5)
    return pose


def run_goal(navigator: BasicNavigator, x: float, y: float, yaw_rad: float) -> bool:
    goal_pose = make_pose(navigator, x, y, yaw_rad)
    print(f"[auto-nav] sending goal x={x:.2f} y={y:.2f} yaw={yaw_rad:.2f}")
    navigator.goToPose(goal_pose)

    last_log = 0.0
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        now = time.monotonic()
        if feedback is not None and now - last_log >= 2.0:
            distance = getattr(feedback, "distance_remaining", None)
            eta_ns = getattr(feedback, "estimated_time_remaining", None)
            if distance is not None:
                if eta_ns is not None:
                    eta_sec = eta_ns.nanosec / 1e9 + eta_ns.sec
                    print(
                        f"[auto-nav] remaining={distance:.2f}m eta={eta_sec:.1f}s"
                    )
                else:
                    print(f"[auto-nav] remaining={distance:.2f}m")
            last_log = now
        time.sleep(0.1)

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("[auto-nav] goal succeeded")
        return True
    if result == TaskResult.CANCELED:
        print("[auto-nav] goal canceled")
        return False
    print("[auto-nav] goal failed")
    return False


def main() -> int:
    rclpy.init()
    navigator = BasicNavigator()

    try:
        initial_pose = make_pose(navigator, 0.0, 0.0, 0.0)
        print("[auto-nav] setting initial pose to map origin")
        navigator.setInitialPose(initial_pose)
        print("[auto-nav] waiting for Nav2 to become active")
        navigator.waitUntilNav2Active()

        # Give AMCL and the planners a moment after activation.
        time.sleep(2.0)

        candidate_goals = [
            (1.0, 0.5, 0.0),
            (1.0, -0.5, 0.0),
            (0.5, 0.0, 0.0),
        ]

        for x, y, yaw in candidate_goals:
            if run_goal(navigator, x, y, yaw):
                print("[auto-nav] leaving simulation running for the Copper observer")
                while True:
                    time.sleep(1.0)

        print("[auto-nav] no candidate goal succeeded", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 0
    finally:
        navigator.destroyNode()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
