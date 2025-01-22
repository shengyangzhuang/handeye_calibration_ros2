"""
Copyright © 2024 Shengyang Zhuang. All rights reserved.

Contact: https://shengyangzhuang.github.io/
"""
import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint
import math
import time

class LBRJointTrajectoryExecutionerNode(Node):
    def __init__(
        self,
        node_name: str,
    ) -> None:
        super().__init__(node_name=node_name)
        self.joint_trajectory_action_client_ = ActionClient(
            node=self,
            action_type=FollowJointTrajectory,
            action_name="/lbr/joint_trajectory_controller/follow_joint_trajectory",
        )
        while not self.joint_trajectory_action_client_.wait_for_server(1):
            self.get_logger().info("Waiting for action server to become available...")
        self.get_logger().info("Action server available.")

    def execute(self, positions: list, sec_from_start: int = 15):
        if len(positions) != 7:
            self.get_logger().error("Invalid number of joint positions.")
            return

        joint_trajectory_goal = FollowJointTrajectory.Goal()
        goal_sec_tolerance = 1
        joint_trajectory_goal.goal_time_tolerance.sec = goal_sec_tolerance

        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * len(positions)
        point.time_from_start.sec = sec_from_start

        for i in range(7):
            joint_trajectory_goal.trajectory.joint_names.append(f"A{i + 1}")

        joint_trajectory_goal.trajectory.points.append(point)

        # send goal
        goal_future = self.joint_trajectory_action_client_.send_goal_async(
            joint_trajectory_goal
        )
        rclpy.spin_until_future_complete(self, goal_future)
        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected by server.")
            return
        self.get_logger().info("Goal was accepted by server.")

        # wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=sec_from_start + goal_sec_tolerance
        )

        if (
            result_future.result().result.error_code
            != FollowJointTrajectory.Result.SUCCESSFUL
        ):
            self.get_logger().error("Failed to execute joint trajectory.")
            return


def main(args: list = None) -> None:
    rclpy.init(args=args)
    joint_trajectory_executioner_node = LBRJointTrajectoryExecutionerNode(
        "joint_trajectory_executioner_node"
    )

    poses_degrees = [
        #[  43.,   61.,  -30.,  -55.,   0.,  95.,    0.], #
        #[   0.,   56.,    0.,  -61.,    0.,   95.,    0.],#
        #[ -35.,   53.,   14.,  -63.,  0.,  93.,    0.], #
        #[  -40.,   51.,  0.,  -79.,  32.,   85.,   -47.], #
        [   0.,    16.,    0.,  -80.,    0.,   85.,    0.],#
        #[ 31.,   44.,   0.,  -76.,   -27.,   90.,  50.], #
        #[  38.,  8.,  18., -108.,    -36.,   74.,  54.], #
        #[   0.,  -50.,    0., -110.,    0.,   85.,    0.],#
        #[ -70.,  0.,   18., -118.,    44.,   74.,   -63.],
        # additional positions
        #[ 31, 51, -42, -73, 18, 85, 0],
        #[ -15, 57, -18, -73, 41, 94, -52],
        #[ 0, 0, -51, -90, 42, 94, 0],
        #[ 0, 0, 40, -90, -42, 94, 0],
        #[ 0, -68, -37, -120, 6, 88, 0],
        #[ 0, -52, 44, -120, -14, 80, 0]
    ]

    # Convert each pose from degrees to radians
    poses = [
        [math.radians(angle) for angle in pose] for pose in poses_degrees
    ]


    # Execute each pose with a 5 second pause in between
    for pose in poses:
        joint_trajectory_executioner_node.get_logger().info(f"Moving to pose: {pose}")
        joint_trajectory_executioner_node.execute(pose)
        time.sleep(5)  # Pause for 5 seconds

    rclpy.shutdown()