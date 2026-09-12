import os
import sys
import unittest
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import time

def generate_test_description():
    lidar_subscriber_node = launch_ros.actions.Node(
        package='lidar_mapper',
        executable='lidar_subscriber',
        output='screen'
    )

    return launch.LaunchDescription([
        lidar_subscriber_node,
        launch_testing.actions.ReadyToTest()
    ]), {'lidar_subscriber_node': lidar_subscriber_node}

class TestIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # Clean up CSV file if exists
        cls.csv_file = 'pothole_map_data.csv'
        if os.path.exists(cls.csv_file):
            os.remove(cls.csv_file)

    def test_pothole_logging(self, proc_info, proc_output):
        import rclpy
        from sensor_msgs.msg import LaserScan, NavSatFix
        import math

        rclpy.init()
        node = rclpy.create_node('test_publisher')

        scan_pub = node.create_publisher(LaserScan, '/scan', 10)
        gps_pub = node.create_publisher(NavSatFix, '/gps/fix', 10)

        # Wait for the node to be ready and subscribers to connect
        time.sleep(2)

        # 1. Publish baseline scan
        scan_msg = LaserScan()
        scan_msg.header.stamp = node.get_clock().now().to_msg()
        scan_msg.angle_min = -math.pi
        scan_msg.angle_max = math.pi
        scan_msg.angle_increment = math.pi / 180.0
        scan_msg.range_min = 0.0
        scan_msg.range_max = 100.0
        scan_msg.ranges = [10.0] * 361

        gps_msg = NavSatFix()
        gps_msg.header.stamp = scan_msg.header.stamp
        gps_msg.latitude = 12.34
        gps_msg.longitude = 56.78

        scan_pub.publish(scan_msg)
        gps_pub.publish(gps_msg)

        time.sleep(0.5)

        # 2. Publish pothole scan
        scan_msg.header.stamp = node.get_clock().now().to_msg()
        gps_msg.header.stamp = scan_msg.header.stamp

        scan_msg.ranges = [10.0] * 361
        scan_msg.ranges[180] = 10.10 # 10cm dip

        scan_pub.publish(scan_msg)
        gps_pub.publish(gps_msg)

        time.sleep(1)

        rclpy.shutdown()

        # Check if CSV exists and has content
        self.assertTrue(os.path.exists(self.csv_file))

        with open(self.csv_file, 'r') as f:
            lines = f.readlines()

        # Header + at least 1 pothole log
        self.assertGreaterEqual(len(lines), 2)

        # Verify the logged row
        data_row = lines[1].strip().split(',')
        # timestamp, lat, lon, depth
        self.assertEqual(float(data_row[1]), 12.34)
        self.assertEqual(float(data_row[2]), 56.78)
        self.assertEqual(float(data_row[3]), 0.100) # 10.10 - 10.0 = 0.1
