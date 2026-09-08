import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, NavSatFix
import message_filters
import math
import csv
import os

class LidarSubscriber(Node):

    def __init__(self):
        super().__init__('lidar_subscriber')

        # Create message filter subscribers
        self.lidar_sub = message_filters.Subscriber(self, LaserScan, '/scan')
        self.gps_sub = message_filters.Subscriber(self, NavSatFix, '/gps/fix')

        # Synchronize based on timestamps
        self.ts = message_filters.ApproximateTimeSynchronizer([self.lidar_sub, self.gps_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.listener_callback)

        self.ema_baseline = None
        self.alpha = 0.1 # EMA smoothing factor
        self.last_detection_time = 0.0
        self.csv_file = 'pothole_map_data.csv'

        # Initialize CSV file with headers if it doesn't exist
        if not os.path.exists(self.csv_file):
            with open(self.csv_file, mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'latitude', 'longitude', 'depth_m'])

    def listener_callback(self, scan_msg, gps_msg):
        # Extract timestamp in seconds
        current_time = scan_msg.header.stamp.sec + scan_msg.header.stamp.nanosec * 1e-9

        # Calculate indices for the central 90 degrees (which is pi/2 radians)
        # LaserScan is from angle_min to angle_max with angle_increment
        # Assume 0 is straight ahead, or the center of the scan is straight ahead.
        # Usually, angle_min and angle_max are symmetric (e.g. -pi to pi).
        # Central angle is (angle_max + angle_min) / 2
        # We want central angle +/- 45 degrees (pi/4)

        mid_angle = (scan_msg.angle_max + scan_msg.angle_min) / 2.0
        target_min_angle = mid_angle - math.pi / 4.0
        target_max_angle = mid_angle + math.pi / 4.0

        # Determine the start and end index
        if scan_msg.angle_increment > 0:
            start_index = max(0, int((target_min_angle - scan_msg.angle_min) / scan_msg.angle_increment))
            end_index = min(len(scan_msg.ranges) - 1, int((target_max_angle - scan_msg.angle_min) / scan_msg.angle_increment))
        else:
            self.get_logger().warn("angle_increment should be positive")
            return

        central_ranges = []
        for i in range(start_index, end_index + 1):
            r = scan_msg.ranges[i]
            if not math.isnan(r) and not math.isinf(r) and scan_msg.range_min <= r <= scan_msg.range_max:
                central_ranges.append(r)

        if not central_ranges:
            self.get_logger().info('No valid laser points found in central 90 degrees.')
            return

        # Calculate the current average distance in the central FOV
        current_avg = sum(central_ranges) / len(central_ranges)

        # Update EMA baseline
        if self.ema_baseline is None:
            self.ema_baseline = current_avg
        else:
            self.ema_baseline = self.alpha * current_avg + (1 - self.alpha) * self.ema_baseline

        # Pothole detection: check if any measurement in central FOV is > baseline + 0.05
        # Or should we check the average? The prompt says: "when a measurement exceeds this baseline by more than 5cm"
        max_dip = 0.0
        for r in central_ranges:
            dip = r - self.ema_baseline
            if dip > max_dip:
                max_dip = dip

        if max_dip > 0.05: # > 5cm
            # Check lockout
            if current_time - self.last_detection_time >= 0.5:
                # Log to CSV
                with open(self.csv_file, mode='a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([f"{current_time:.6f}", gps_msg.latitude, gps_msg.longitude, f"{max_dip:.3f}"])
                self.last_detection_time = current_time
                self.get_logger().info(f'Pothole detected! Depth: {max_dip:.3f}m')
        else:
            self.get_logger().info(f'Average valid distance: {current_avg:.3f}m')

def main(args=None):
    rclpy.init(args=args)
    lidar_subscriber = LidarSubscriber()
    rclpy.spin(lidar_subscriber)
    lidar_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
