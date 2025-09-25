#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import sys, select


class GoalCollectorExecutor(Node):
    def __init__(self):
        super().__init__('goal_collector_executor')
        self.goals = []
        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        print("✅ GoalCollectorExecutor started. Click multiple goals in RViz (2D Goal Pose tool).")
        print("👉 When finished, press ENTER in the terminal to start execution.")

    def goal_callback(self, msg: PoseStamped):
        self.goals.append(msg)
        print(f"\n🎯 Stored Goal {len(self.goals)}: "
              f"({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})")

        # Show all saved so far
        print("📌 Saved Goals so far:")
        for i, g in enumerate(self.goals, start=1):
            print(f"  {i}: ({g.pose.position.x:.2f}, {g.pose.position.y:.2f})")


def main(args=None):
    rclpy.init(args=args)
    node = GoalCollectorExecutor()
    navigator = BasicNavigator()

    # Wait for Nav2 to be ready
    navigator.waitUntilNav2Active()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # Collect goals until ENTER is pressed
    print("\n⏸️ Press ENTER to stop collecting and start executing goals...\n")
    while True:
        # Check for keyboard input without blocking
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            _ = sys.stdin.readline()
            break
        executor.spin_once(timeout_sec=0.1)

    # Stop collecting
    executor.remove_node(node)
    node.destroy_node()

    # Execute saved goals
    print("\n🚀 Executing stored goals...")
    for i, g in enumerate(node.goals, start=1):
        g.header.stamp = navigator.get_clock().now().to_msg()
        print(f"➡️ Navigating to Goal {i}: ({g.pose.position.x:.2f}, {g.pose.position.y:.2f})")

        navigator.goToPose(g)

        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback:
                print(f"[Goal {i}] Distance remaining: {feedback.distance_remaining:.2f} m")

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"✅ Goal {i} reached successfully!")
        elif result == TaskResult.CANCELED:
            print(f"⚠️ Goal {i} was canceled.")
        elif result == TaskResult.FAILED:
            print(f"❌ Goal {i} failed.")

    print("🎉 All clicked goals executed.")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
