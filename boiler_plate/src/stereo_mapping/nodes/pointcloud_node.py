import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import struct
import yaml
import os


class PointCloudNode(Node):
    def __init__(self):
        super().__init__('pointcloud_node')
        
        self.bridge = CvBridge()
        self.K_left = np.eye(3)  # Default camera matrix
        
        # Load calibration to get camera intrinsics
        config_dir = os.path.join(
            os.path.dirname(__file__), '../../stereo_calibration/config'
        )
        calib_file = os.path.join(config_dir, 'stereo.yaml')
        
        if os.path.exists(calib_file):
            try:
                with open(calib_file, 'r') as f:
                    calib_params = yaml.safe_load(f)
                    self.K_left = np.array(calib_params.get('K_left', np.eye(3)))
                    self.get_logger().info("Loaded camera intrinsics from calibration file")
            except Exception as e:
                self.get_logger().warn(f"Could not load calibration file: {e}")
        else:
            self.get_logger().warn(f"Calibration file not found at {calib_file}")
            self.get_logger().info("Using default camera matrix")

        self.sub = self.create_subscription(
            Image, '/stereo/depth', self.cb, 10)

        self.pub = self.create_publisher(
            PointCloud2, '/stereo/points', 10)

        self.get_logger().info("PointCloud node started")

    def cb(self, msg):
        """Convert depth map to point cloud"""
        try:
            # Convert ROS Image to OpenCV format (uint16, values in mm)
            depth_image = self.bridge.imgmsg_to_cv2(msg, "mono16")
            
            # Convert from uint16 (mm) to float32 (m)
            depth_float = depth_image.astype(np.float32) / 1000.0
            
            # Generate point cloud
            pointcloud = self.depth_to_pointcloud(depth_float)
            
            # Convert to ROS PointCloud2 message
            pc2_msg = self.pointcloud_to_ros(pointcloud, msg.header)
            
            self.pub.publish(pc2_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error processing depth image: {e}")

    def depth_to_pointcloud(self, depth):
        """
        Convert depth map to 3D point cloud.
        
        Args:
            depth: HxW depth map in meters
            
        Returns:
            Nx3 array of 3D points (x, y, z) in meters
        """
        height, width = depth.shape
        
        # Get camera intrinsics
        fx = self.K_left[0, 0]
        fy = self.K_left[1, 1]
        cx = self.K_left[0, 2]
        cy = self.K_left[1, 2]
        
        # Create coordinate grids
        x_coords = np.arange(width) - cx
        y_coords = np.arange(height) - cy
        xx, yy = np.meshgrid(x_coords, y_coords)
        
        # Back-project to 3D
        # x = (u - cx) * z / fx
        # y = (v - cy) * z / fy
        # z = z
        
        X = xx * depth / fx
        Y = yy * depth / fy
        Z = depth
        
        # Stack into Nx3 array and filter invalid points
        points = np.stack([X, Y, Z], axis=-1)
        points = points.reshape(-1, 3)
        
        # Remove invalid points (zero depth or NaN)
        valid_mask = (points[:, 2] > 0) & np.isfinite(points).all(axis=1)
        points = points[valid_mask]
        
        return points

    def pointcloud_to_ros(self, points, header):
        """
        Convert point cloud array to ROS PointCloud2 message.
        
        Args:
            points: Nx3 array of points
            header: ROS Header for the message
            
        Returns:
            PointCloud2 message
        """
        # Define fields for the point cloud
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Create PointCloud2 message
        pc2 = PointCloud2()
        pc2.header = header
        pc2.header.frame_id = "stereo_frame"
        pc2.height = 1
        pc2.width = len(points)
        pc2.fields = fields
        pc2.is_bigendian = False
        pc2.point_step = 12  # 3 float32 values, 4 bytes each
        pc2.row_step = pc2.point_step * pc2.width
        
        # Serialize point data
        point_data = []
        for point in points:
            x, y, z = point
            point_data.append(struct.pack('fff', x, y, z))
        
        pc2.data = b''.join(point_data)
        
        return pc2


def main():
    rclpy.init()
    node = PointCloudNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()