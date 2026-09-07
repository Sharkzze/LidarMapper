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

        self.running_baseline = None
        self.last_pothole_time_sec = 0.0

    def listener_callback(self, msg, gps_msg):
        # Calculate central 30 degrees (-15 to +15 degrees)
        # Convert degrees to radians
        min_angle_rad = math.radians(-15.0)
        max_angle_rad = math.radians(15.0)

        valid_central_ranges = []
        valid_central_angles = []

        for i, r in enumerate(msg.ranges):
            # Calculate angle for this range
            angle = msg.angle_min + i * msg.angle_increment

            # Check if within central 30 degrees (handle potential wrap-around, though typically angle_min is -pi and max is pi)
            # We assume a front facing lidar where 0 is forward. If 0 is not forward, we might need to adjust.
            # Usually -15 to +15 degrees is around 0.
            if min_angle_rad <= angle <= max_angle_rad:
                if not math.isnan(r) and not math.isinf(r) and msg.range_min <= r <= msg.range_max:
                    valid_central_ranges.append(r)
                    valid_central_angles.append(angle)

        if valid_central_ranges:
            current_avg = sum(valid_central_ranges) / len(valid_central_ranges)

            # Update running baseline
            if self.running_baseline is None:
                self.running_baseline = current_avg
            else:
                self.running_baseline = 0.9 * self.running_baseline + 0.1 * current_avg

            self.get_logger().info(f'Running baseline distance: {self.running_baseline:.3f}m')

            # Check for potholes
            current_time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

            # 0.5-second time-based lockout
            if current_time_sec - self.last_pothole_time_sec >= 0.5:
                for r, angle in zip(valid_central_ranges, valid_central_angles):
                    # Check if depth is > 5cm (0.05m)
                    if r > self.running_baseline + 0.05:
                        depth = r - self.running_baseline
                        angle_deg = math.degrees(angle)
                        self.get_logger().info(f'Pothole detected at angle {angle_deg:.1f} degrees! Depth: {depth:.3f}m')
                        self.last_pothole_time_sec = current_time_sec
                        break # Only log one pothole per scan
        else:
            self.get_logger().info('No valid laser points found in central 30 degrees.')

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
