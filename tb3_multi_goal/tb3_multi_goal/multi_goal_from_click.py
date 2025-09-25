#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class GoalCollector(Node):
    def __init__(self):
        super().__init__('goal_collector')

        # ✅ Set use_sim_time instead of redeclaring it
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time',
            rclpy.Parameter.Type.BOOL,
            True
        )])

        self.goals = []
        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',   # Topic published when you click in RViz
            self.goal_callback,
            10
        )
        print("✅ GoalCollector started. Click goals in RViz (2D Nav Goal tool).")

    def goal_callback(self, msg: PoseStamped):
        self.goals.append(msg)
        print(f"🎯 Collected Goal {len(self.goals)}: "
              f"({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})")


def main(args=None):
    rclpy.init(args=args)
    node = GoalCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print("\nShutting down. Collected goals:")
        for i, g in enumerate(node.goals, start=1):
            print(f"  Goal {i}: ({g.pose.position.x:.2f}, {g.pose.position.y:.2f})")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
