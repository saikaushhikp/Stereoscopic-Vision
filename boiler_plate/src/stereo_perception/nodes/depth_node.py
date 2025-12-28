import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
import os


class DepthNode(Node):
    def __init__(self):
        super().__init__('depth_node')
        
        self.bridge = CvBridge()
        self.focal_length = 700.0  # Default value
        self.baseline = 0.12  # Default value in meters
        
        # Load calibration parameters to get focal length and baseline
        config_dir = os.path.join(
            os.path.dirname(__file__), '../../stereo_calibration/config'
        )
        calib_file = os.path.join(config_dir, 'stereo.yaml')
        
        if os.path.exists(calib_file):
            try:
                with open(calib_file, 'r') as f:
                    calib_params = yaml.safe_load(f)
                    self.focal_length = float(calib_params.get('focal_length', self.focal_length))
                    self.baseline = float(calib_params.get('baseline', self.baseline))
                    self.get_logger().info(
                        f"Loaded calibration: focal_length={self.focal_length}, baseline={self.baseline}"
                    )
            except Exception as e:
                self.get_logger().warn(f"Could not load calibration file: {e}")
                self.get_logger().info(f"Using default values: focal_length={self.focal_length}, baseline={self.baseline}")
        else:
            self.get_logger().warn(f"Calibration file not found at {calib_file}")
            self.get_logger().info(f"Using default values: focal_length={self.focal_length}, baseline={self.baseline}")

        self.sub = self.create_subscription(
            Image, '/stereo/disparity', self.cb, 10)

        self.pub = self.create_publisher(
            Image, '/stereo/depth', 10)

        self.get_logger().info("Depth node started")

    def cb(self, msg):
        """Convert disparity map to depth map"""
        try:
            # Convert disparity image to OpenCV format
            disparity_normalized = self.bridge.imgmsg_to_cv2(msg, "mono8")
            
            # Denormalize disparity (was normalized to 0-255 in disparity_node)
            # We need to scale it back to pixel disparities
            disparity = disparity_normalized.astype(np.float32)
            disparity_max = disparity.max()
            if disparity_max > 0:
                disparity = (disparity / 255.0) * disparity_max
            
            # Compute depth using the stereo formula:
            # depth = (baseline * focal_length) / disparity
            # depth in meters
            
            # Avoid division by zero
            with np.errstate(divide='ignore', invalid='ignore'):
                depth = np.where(
                    disparity > 0,
                    (self.baseline * self.focal_length) / disparity,
                    0
                )
            
            # Clip depth values to reasonable range
            # Typically 0.1m to 10m for indoor robots
            depth = np.clip(depth, 0.1, 10.0)
            
            # Convert to 16-bit unsigned integer for ROS Image message
            # Scale: 1 pixel = 1mm, so multiply by 1000 then convert to uint16
            depth_uint16 = (depth * 1000).astype(np.uint16)
            
            # Create ROS Image message
            depth_msg = self.bridge.cv2_to_imgmsg(depth_uint16, encoding="mono16")
            depth_msg.header = msg.header
            depth_msg.header.frame_id = "stereo_frame"
            
            self.pub.publish(depth_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error processing disparity: {e}")

    def set_calibration(self, focal_length, baseline):
        """Update calibration parameters (e.g., from dynamic reconfigure)"""
        self.focal_length = focal_length
        self.baseline = baseline
        self.get_logger().info(
            f"Updated calibration: focal_length={self.focal_length}, baseline={self.baseline}"
        )


def main():
    rclpy.init()
    node = DepthNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
