import sys
from unittest.mock import MagicMock, patch, mock_open, ANY
import math

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

def create_mock_scan(ranges, angle_min=-math.pi, angle_max=math.pi, angle_increment=math.pi/180, time_sec=0):
    msg = MagicMock()
    msg.ranges = ranges
    msg.angle_min = angle_min
    msg.angle_max = angle_max
    msg.angle_increment = angle_increment
    msg.range_min = 0.0
    msg.range_max = 100.0
    msg.header.stamp.sec = time_sec
    msg.header.stamp.nanosec = 0
    return msg

def create_mock_gps(lat=0.0, lon=0.0):
    msg = MagicMock()
    msg.latitude = lat
    msg.longitude = lon
    return msg

@patch('os.path.exists', return_value=True)
def test_listener_callback_empty(mock_exists):
    node = lidar_subscriber.LidarSubscriber()
    logger_mock = MagicMock()
    node.get_logger = MagicMock(return_value=logger_mock)

    # Empty ranges but valid angles
    msg = create_mock_scan([])
    gps_msg = create_mock_gps()

    node.listener_callback(msg, gps_msg)

    logger_mock.info.assert_called_with('No valid laser points found in central 90 degrees.')

@patch('os.path.exists', return_value=True)
def test_listener_callback_valid_ranges(mock_exists):
    node = lidar_subscriber.LidarSubscriber()
    logger_mock = MagicMock()
    node.get_logger = MagicMock(return_value=logger_mock)

    ranges = [10.0] * 361
    ranges[180] = 9.98

    msg = create_mock_scan(ranges, angle_min=-math.pi, angle_max=math.pi, angle_increment=math.pi/180)
    gps_msg = create_mock_gps()

    node.listener_callback(msg, gps_msg)

    expected_avg = (10.0 * 90 + 9.98) / 91

    assert node.ema_baseline == expected_avg
    logger_mock.info.assert_called_with(ANY)
    assert f'Average valid distance:' in logger_mock.info.call_args[0][0]

@patch('os.path.exists', return_value=True)
@patch('builtins.open', new_callable=mock_open)
def test_pothole_detection_and_lockout(mock_file, mock_exists):
    node = lidar_subscriber.LidarSubscriber()
    logger_mock = MagicMock()
    node.get_logger = MagicMock(return_value=logger_mock)

    # First message establishes baseline
    ranges1 = [10.0] * 361
    msg1 = create_mock_scan(ranges1, time_sec=1)
    gps_msg1 = create_mock_gps()
    node.listener_callback(msg1, gps_msg1)
    assert node.ema_baseline == 10.0

    # Second message introduces a dip of 0.06m (6cm), which triggers pothole > 0.05m
    ranges2 = [10.0] * 361
    ranges2[180] = 10.06
    msg2 = create_mock_scan(ranges2, time_sec=2)
    gps_msg2 = create_mock_gps(lat=1.0, lon=2.0)

    node.listener_callback(msg2, gps_msg2)
    logger_mock.info.assert_called_with(f'Pothole detected! Depth: {0.060:.3f}m')
    assert node.last_detection_time == 2.0

    # Verify CSV write
    mock_file().write.assert_called()

    # Third message, another dip right after (time_sec=2.2 < 0.5s lockout), should be ignored
    logger_mock.reset_mock()
    ranges3 = [10.0] * 361
    ranges3[180] = 10.06
    msg3 = create_mock_scan(ranges3, time_sec=2.2)
    node.listener_callback(msg3, gps_msg2)

    # Ensure info wasn't called with pothole detected due to lockout
    for call in logger_mock.info.call_args_list:
        assert 'Pothole detected!' not in call[0][0]

    # Fourth message, time_sec=3.0, should detect again
    ranges4 = [10.0] * 361
    ranges4[180] = 10.08
    msg4 = create_mock_scan(ranges4, time_sec=3.0)
    node.listener_callback(msg4, gps_msg2)

    logger_mock.info.assert_called_with(f'Pothole detected! Depth: {0.080:.3f}m')

