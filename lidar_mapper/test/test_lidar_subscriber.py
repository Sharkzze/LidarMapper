import sys
from unittest.mock import MagicMock, patch

# Mock ROS 2 dependencies before importing the module
sys.modules['rclpy'] = MagicMock()
sys.modules['rclpy.node'] = MagicMock()
sys.modules['sensor_msgs'] = MagicMock()
sys.modules['sensor_msgs.msg'] = MagicMock()
sys.modules['message_filters'] = MagicMock()

# Mock the Node class
class MockNode:
    def __init__(self, name):
        self.name = name

    def get_logger(self):
        pass

sys.modules['rclpy.node'].Node = MockNode

import lidar_mapper.lidar_subscriber as lidar_subscriber

def test_listener_callback_empty():
    node = lidar_subscriber.LidarSubscriber()

    # Replace logger with our own mock to assert
    node.get_logger = MagicMock()
    logger_mock = MagicMock()
    node.get_logger.return_value = logger_mock

    # Create mock messages
    msg = MagicMock()
    msg.ranges = []
    msg.range_min = 0.0
    msg.range_max = 10.0

    gps_msg = MagicMock()

    node.listener_callback(msg, gps_msg)

    logger_mock.info.assert_called_with('No valid laser points found.')

def test_listener_callback_valid_ranges():
    node = lidar_subscriber.LidarSubscriber()

    # Replace logger with our own mock to assert
    node.get_logger = MagicMock()
    logger_mock = MagicMock()
    node.get_logger.return_value = logger_mock

    # Create mock messages
    msg = MagicMock()
    # Create ranges: 1.0, inf, NaN, 2.0, out of bounds
    msg.ranges = [1.0, float('inf'), float('nan'), 2.0, -1.0, 11.0]
    msg.range_min = 0.0
    msg.range_max = 10.0

    gps_msg = MagicMock()

    node.listener_callback(msg, gps_msg)

    # Only 1.0 and 2.0 should be valid. Average is 1.5.
    logger_mock.info.assert_called_with('Average valid distance: 1.500m')
