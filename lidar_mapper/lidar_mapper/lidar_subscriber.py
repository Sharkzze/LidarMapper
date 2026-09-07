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

            if downsampled_count > 3:
                # Phase 4: Ground Segmentation using RANSAC
                plane_model, inliers = downsampled_cloud.segment_plane(distance_threshold=0.02,
                                                                       ransac_n=3,
                                                                       num_iterations=1000)
                [a, b, c, d] = plane_model

                # Outliers from the plane are potential obstacles or potholes
                outlier_cloud = downsampled_cloud.select_by_index(inliers, invert=True)

                # Find potholes: points that are below the plane
                # Distance to plane: (ax + by + cz + d) / sqrt(a^2 + b^2 + c^2)
                # We expect the normal (a, b, c) to point upwards (if not, flip it)
                if c < 0:
                    a, b, c, d = -a, -b, -c, -d

                outlier_points = np.asarray(outlier_cloud.points)
                if len(outlier_points) > 0:
                    # Calculate signed distance to the plane
                    distances = (a * outlier_points[:, 0] + b * outlier_points[:, 1] + c * outlier_points[:, 2] + d) / np.sqrt(a**2 + b**2 + c**2)

                    # Potholes are points with distance < -0.03m (below the ground plane by at least 3cm)
                    pothole_indices = np.where(distances < -0.03)[0]
                    pothole_cloud = outlier_cloud.select_by_index(pothole_indices)

                    pothole_count = len(pothole_cloud.points)

                    valid_potholes = 0
                    max_depth = 0.0

                    if pothole_count > 0:
                        # Phase 5: DBSCAN Clustering
                        labels = np.array(pothole_cloud.cluster_dbscan(eps=0.1, min_points=10, print_progress=False))
                        max_label = labels.max()

                        if max_label > -1:
                            for i in range(max_label + 1):
                                cluster_indices = np.where(labels == i)[0]
                                cluster_cloud = pothole_cloud.select_by_index(cluster_indices)

                                # Compute Convex Hull for the cluster
                                try:
                                    hull, _ = cluster_cloud.compute_convex_hull()
                                    area = hull.get_surface_area()

                                    # Filter out small clusters (less than 0.1 m^2 surface area)
                                    if area >= 0.1:
                                        valid_potholes += 1

                                        # Calculate max depth for this cluster
                                        cluster_points = np.asarray(cluster_cloud.points)
                                        cluster_distances = (a * cluster_points[:, 0] + b * cluster_points[:, 1] + c * cluster_points[:, 2] + d) / np.sqrt(a**2 + b**2 + c**2)
                                        # Distance is negative (below plane), so depth is absolute distance
                                        cluster_max_depth = abs(np.min(cluster_distances))

                                        if cluster_max_depth > max_depth:
                                            max_depth = cluster_max_depth
                                except Exception as e:
                                    # Convex hull might fail if points are coplanar or collinear, ignore
                                    pass

                    self.get_logger().info(f'Original: {original_count}, Downsampled: {downsampled_count}, Pothole Points: {pothole_count}')
                    self.get_logger().info(f'Total valid potholes: {valid_potholes}, Max depth: {max_depth:.3f}m')
                else:
                    self.get_logger().info(f'Original: {original_count}, Downsampled: {downsampled_count}, Pothole Points: 0')
                    self.get_logger().info('Total valid potholes: 0, Max depth: 0.000m')
            else:
                self.get_logger().info(f'Original: {original_count}, Downsampled: {downsampled_count}')
        else:
            downsampled_count = 0
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
