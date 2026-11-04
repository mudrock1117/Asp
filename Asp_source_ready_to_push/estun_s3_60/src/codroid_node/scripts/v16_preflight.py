#!/usr/bin/env python3
"""V16 real-robot preflight: observation only, never publishes robot commands.

Run beside the existing CodroidIO node to verify that the ROS interfaces needed
for V16 are present and fresh before a motion controller is introduced.
"""
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from codroid_msgs.msg import RobotInfo


EXPECTED_JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")


class V16Preflight(Node):
    def __init__(self):
        super().__init__("v16_preflight")
        self.declare_parameter("target_topic", "/v16/target_point_base")
        self.declare_parameter("max_joint_age_s", 0.25)
        self.declare_parameter("max_target_age_s", 0.50)
        self.target_topic = self.get_parameter("target_topic").value
        self.max_joint_age_s = float(self.get_parameter("max_joint_age_s").value)
        self.max_target_age_s = float(self.get_parameter("max_target_age_s").value)

        self.joint_state = None
        self.robot_info = None
        self.target = None
        self.joint_at = None
        self.robot_at = None
        self.target_at = None
        self.status_pub = self.create_publisher(String, "/v16/preflight_status", 10)
        self.joint_sub = self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)
        self.robot_sub = self.create_subscription(RobotInfo, "RobotInfo", self.on_robot_info, 10)
        self.target_sub = self.create_subscription(PointStamped, self.target_topic, self.on_target, 10)
        self.status_timer = self.create_timer(0.2, self.publish_status)
        self.get_logger().info(
            "V16 preflight started: observation-only; it never publishes RobotMove or /joint_goal_states."
        )

    def on_joint_state(self, msg):
        self.joint_state = msg
        self.joint_at = time.monotonic()

    def on_robot_info(self, msg):
        self.robot_info = msg
        self.robot_at = time.monotonic()

    def on_target(self, msg):
        self.target = msg
        self.target_at = time.monotonic()

    @staticmethod
    def age(now, received_at):
        return math.inf if received_at is None else now - received_at

    def publish_status(self):
        now = time.monotonic()
        joint_age = self.age(now, self.joint_at)
        target_age = self.age(now, self.target_at)
        issues = []

        if self.joint_state is None:
            issues.append("no /joint_states")
        elif joint_age > self.max_joint_age_s:
            issues.append(f"stale /joint_states ({joint_age:.2f}s)")
        elif tuple(self.joint_state.name) != EXPECTED_JOINTS:
            issues.append(f"unexpected joint names {list(self.joint_state.name)}")
        elif len(self.joint_state.position) != 6 or not all(math.isfinite(v) for v in self.joint_state.position):
            issues.append("invalid joint positions")

        if self.robot_info is None:
            issues.append("no RobotInfo")
        elif int(self.robot_info.fault_flag) != 0:
            issues.append(f"controller fault_flag={self.robot_info.fault_flag}")
        # AutoReady is sufficient for the observation-only shadow phase.  A
        # later node that can command the arm must require AutoRunning again.
        elif self.robot_info.state not in ("AutoReady", "AutoRunning"):
            issues.append(
                f"controller state={self.robot_info.state!r}, expected 'AutoReady' or 'AutoRunning'"
            )

        if self.target is None:
            issues.append(f"no target on {self.target_topic}")
        elif target_age > self.max_target_age_s:
            issues.append(f"stale target ({target_age:.2f}s)")
        elif self.target.header.frame_id not in ("base", "base_link"):
            issues.append(f"target frame={self.target.header.frame_id!r}, expected base/base_link")
        elif not all(math.isfinite(v) for v in (self.target.point.x, self.target.point.y, self.target.point.z)):
            issues.append("invalid target coordinates")

        payload = String()
        payload.data = "READY_FOR_SHADOW" if not issues else "BLOCKED: " + "; ".join(issues)
        self.status_pub.publish(payload)
        self.get_logger().info(payload.data)


def main():
    rclpy.init()
    node = V16Preflight()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
