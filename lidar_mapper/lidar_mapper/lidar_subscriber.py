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

            # Apply pass-through crop filter (remove Z > 1.5m)
            # Find points where Z <= 1.5
            bbox_min = np.array([-np.inf, -np.inf, -np.inf])
            bbox_max = np.array([np.inf, np.inf, 1.5])
            bounding_box = o3d.geometry.AxisAlignedBoundingBox(bbox_min, bbox_max)
            cropped_cloud = o3d_cloud.crop(bounding_box)

            # Apply voxel grid downsampling with voxel size 0.05m
            downsampled_cloud = cropped_cloud.voxel_down_sample(voxel_size=0.05)

            downsampled_count = len(downsampled_cloud.points)
        else:
            downsampled_count = 0

        # Log the counts
        self.get_logger().info(f'Original point count: {original_count}, Downsampled point count: {downsampled_count}')


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
