#!/usr/bin/env python3

import rclpy
import yaml
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from pathlib import Path


def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Wait for Nav2 to start
    navigator.waitUntilNav2Active()

    # Load goals from YAML
    yaml_file = Path(__file__).resolve().parent.parent / "goals.yaml"
    with open(yaml_file, 'r') as f:
        data = yaml.safe_load(f)

    goals = data.get("goals", [])
    if not goals:
        print("⚠️ No goals found in YAML file.")
        return

    # Convert to PoseStamped
    goal_poses = []
    for g in goals:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = g["x"]
        pose.pose.position.y = g["y"]
        pose.pose.orientation.w = g.get("w", 1.0)
        goal_poses.append(pose)

    # Navigate through all goals
    for i, goal in enumerate(goal_poses, start=1):
        print(f"🚀 Navigating to Goal {i} at ({goal.pose.position.x}, {goal.pose.position.y})")
        navigator.goToPose(goal)

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
            print(f"❌ Goal {i} failed. Skipping...")

    print("🎉 All YAML goals processed.")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
