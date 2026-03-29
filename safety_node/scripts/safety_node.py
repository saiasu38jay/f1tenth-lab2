#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


class SafetyNode(Node):
    """
    Automatic Emergency Braking node using instantaneous Time to Collision (iTTC).
    """

    def __init__(self):
        super().__init__('safety_node')

        self.speed = 0.0
        self.ttc_threshold = 0.8
        self.min_valid_range = 0.01

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10
        )

        self.get_logger().info('Safety node started.')

    def odom_callback(self, odom_msg: Odometry):
        # Longitudinal speed of the vehicle
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg: LaserScan):
        # If car is not moving forward, no need to brake
        if self.speed <= 0.0:
            return

        ranges = np.array(scan_msg.ranges, dtype=np.float64)

        angles = scan_msg.angle_min + np.arange(len(ranges)) * scan_msg.angle_increment

        # Clean up invalid scan values
        valid = np.isfinite(ranges) & (ranges > self.min_valid_range)
        if not np.any(valid):
            return

        # Range rate approximation using current forward speed projected onto each beam
        # r_dot = -v * cos(theta) when approaching an obstacle in front
        range_rates = -self.speed * np.cos(angles)

        # Only beams with negative range rate are actually closing in
        closing_rates = np.maximum(-range_rates, 0.0)

        # Prevent divide-by-zero
        ittc = np.full_like(ranges, np.inf, dtype=np.float64)
        safe_mask = valid & (closing_rates > 1e-6)
        ittc[safe_mask] = ranges[safe_mask] / closing_rates[safe_mask]

        min_ttc = np.min(ittc)

        if min_ttc < self.ttc_threshold:
            brake_msg = AckermannDriveStamped()
            brake_msg.drive.speed = 0.0
            brake_msg.drive.steering_angle = 0.0
            self.drive_pub.publish(brake_msg)
            self.get_logger().warn(f'BRAKE! min iTTC = {min_ttc:.3f}s')


def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
