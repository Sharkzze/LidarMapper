import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, NavSatFix
import message_filters
import math

class LidarSubscriber(Node):

    def __init__(self):
        super().__init__('lidar_subscriber')

        # Create message filter subscribers
        self.lidar_sub = message_filters.Subscriber(self, LaserScan, '/scan')
        self.gps_sub = message_filters.Subscriber(self, NavSatFix, '/gps/fix')

        # Synchronize based on timestamps
        self.ts = message_filters.ApproximateTimeSynchronizer([self.lidar_sub, self.gps_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.listener_callback)

    def listener_callback(self, msg, gps_msg):
        valid_ranges = []
        for r in msg.ranges:
            if not math.isnan(r) and not math.isinf(r) and msg.range_min <= r <= msg.range_max:
                valid_ranges.append(r)

        if valid_ranges:
            avg_distance = sum(valid_ranges) / len(valid_ranges)
            self.get_logger().info(f'Average valid distance: {avg_distance:.3f}m')
        else:
            self.get_logger().info('No valid laser points found.')

def main(args=None):
    rclpy.init(args=args)

    lidar_subscriber = LidarSubscriber()

    rclpy.spin(lidar_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lidar_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
