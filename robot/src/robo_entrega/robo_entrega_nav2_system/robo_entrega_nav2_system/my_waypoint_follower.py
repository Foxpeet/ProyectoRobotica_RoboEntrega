import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
import math

class MyWaypointFollower(Node):

    def __init__(self):
        super().__init__('my_waypoint_follower')
        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self._timer = self.create_timer(1.0, self._send_goal_if_ready)
        self._goal_sent = False

    def _send_goal_if_ready(self):
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Esperando al servidor follow_waypoints...')
            return

        if not self._goal_sent:
            goal_msg = FollowWaypoints.Goal()
            goal_msg.poses = self._create_waypoints()
            self.get_logger().info('Enviando lista de waypoints...')
            self._action_client.send_goal_async(goal_msg)
            self._goal_sent = True

    def _create_waypoints(self):
        waypoints = []

        # Lista de posiciones (x, y, yaw)
        positions = [
            (0.0, 0.0, 0.0),
            (1.5, -5.0, 270.0),
            (0.0, -22.0, 0.0),
            (4.0, -22.0, 90.0),
            (5.0, -6.0, 0.0),
            (9.5, -6.5, 270.0),
            (8.5, -22.5, 0.0),
            (13.0, -22.5, 90.0),
            (14.0, -6.5, 180.0),
        ]

        for x, y, yaw in positions:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            waypoints.append(pose)

        return waypoints


def main(args=None):
    rclpy.init(args=args)
    node = MyWaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
