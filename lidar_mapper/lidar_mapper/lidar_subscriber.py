import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import numpy as np

class LidarSubscriber(Node):

    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/lidar_points',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.saved_sample = False

    def listener_callback(self, msg):
        # Convert PointCloud2 msg to numpy array
        # Get x, y, z fields
        cloud_data = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        points = np.array(cloud_data, dtype=np.float32)

        original_count = points.shape[0]

        # Convert to Open3D PointCloud
        o3d_cloud = o3d.geometry.PointCloud()
        if original_count > 0:
            o3d_cloud.points = o3d.utility.Vector3dVector(points)

            # Apply pass-through crop filter (remove Z > 1.5m and points outside lane boundaries)
            # Keep points where Z <= 1.5m and -3.0m <= Y <= 3.0m (assuming Y is lateral)
            bbox_min = np.array([-np.inf, -3.0, -np.inf])
            bbox_max = np.array([np.inf, 3.0, 1.5])
            bounding_box = o3d.geometry.AxisAlignedBoundingBox(bbox_min, bbox_max)
            cropped_cloud = o3d_cloud.crop(bounding_box)

            # Apply voxel grid downsampling with voxel size 0.05m
            downsampled_cloud = cropped_cloud.voxel_down_sample(voxel_size=0.05)
            downsampled_count = len(downsampled_cloud.points)

            # RANSAC Ground Segmentation
            if downsampled_count >= 3:
                plane_model, inliers = downsampled_cloud.segment_plane(
                    distance_threshold=0.02,
                    ransac_n=3,
                    num_iterations=1000
                )

                road_surface = downsampled_cloud.select_by_index(inliers)
                obstacles_potholes = downsampled_cloud.select_by_index(inliers, invert=True)

                road_count = len(road_surface.points)
                outlier_count = len(obstacles_potholes.points)

                # Save sample frame
                if not self.saved_sample:
                    o3d.io.write_point_cloud("road_surface.pcd", road_surface)
                    o3d.io.write_point_cloud("obstacles_potholes.pcd", obstacles_potholes)
                    self.get_logger().info('Saved sample .pcd files (road_surface.pcd, obstacles_potholes.pcd)')
                    self.saved_sample = True
            else:
                road_count = 0
                outlier_count = 0
        else:
            downsampled_count = 0
            road_count = 0
            outlier_count = 0

        # Log the counts
        self.get_logger().info(
            f'Original: {original_count}, Downsampled: {downsampled_count}, '
            f'Road Surface: {road_count}, Outliers: {outlier_count}'
        )


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
