#!/usr/bin/env python3

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import rclpy


def main():
    rclpy.init()

    # Create navigator
    navigator = BasicNavigator()

    # Wait until Nav2 is fully active
    navigator.waitUntilNav2Active()

    # List of goals
    goal_poses = []

    # Goal 1
    goal1 = PoseStamped()
    goal1.header.frame_id = 'map'
    goal1.header.stamp = navigator.get_clock().now().to_msg()
    goal1.pose.position.x = 2.0
    goal1.pose.position.y = 0.0
    goal1.pose.orientation.w = 1.0
    goal_poses.append(goal1)

    # Goal 2
    goal2 = PoseStamped()
    goal2.header.frame_id = 'map'
    goal2.header.stamp = navigator.get_clock().now().to_msg()
    goal2.pose.position.x = 0.0
    goal2.pose.position.y = 2.0
    goal2.pose.orientation.w = 1.0
    goal_poses.append(goal2)

    # Visit each goal in order
    for i, goal in enumerate(goal_poses, start=1):
        print(f"🚀 Navigating to Goal {i} at ({goal.pose.position.x}, {goal.pose.position.y})")
        navigator.goToPose(goal)

        # Keep checking until done
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
            if feedback:
                print(f"[Goal {i}] Distance remaining: {feedback.distance_remaining:.2f} m")

        # Check result
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"✅ Goal {i} reached successfully!")
        elif result == TaskResult.CANCELED:
            print(f"⚠️ Goal {i} was canceled.")
        elif result == TaskResult.FAILED:
            print(f"❌ Goal {i} failed. Skipping...")

    print("🎉 All goals processed.")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
