import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
from collections import deque


class DisparityNode(Node):
    def __init__(self):
        super().__init__('disparity_node')
        
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        
        # Image buffers
        self.left_image = None
        self.right_image = None
        self.left_stamp = None
        self.right_stamp = None
        
        # Create stereo matcher (using StereoSGBM for better quality)
        # Parameters tuned for typical stereo setups
        self.stereo = cv2.StereoSGBM_create(
            minDisparity=0,
            numDisparities=16*5,  # Must be divisible by 16
            blockSize=5,
            P1=8 * 3 * 5 ** 2,
            P2=32 * 3 * 5 ** 2,
            disp12MaxDiff=1,
            preFilterCap=63,
            uniquenessRatio=15,
            speckleWindowSize=100,
            speckleRange=32,
            mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
        )
        
        # Alternative: StereoBM (faster but lower quality)
        # self.stereo = cv2.StereoBM_create(numDisparities=16*5, blockSize=15)
        
        self.sub_left = self.create_subscription(
            Image, '/left/image_rect', self.left_cb, 10)

        self.sub_right = self.create_subscription(
            Image, '/right/image_rect', self.right_cb, 10)

        self.pub = self.create_publisher(
            Image, '/stereo/disparity', 10)

        self.get_logger().info("Disparity node started")

    def left_cb(self, msg):
        """Callback for left rectified image"""
        try:
            with self.lock:
                self.left_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
                self.left_stamp = msg.header.stamp
                
            # Try to compute disparity if both images are available
            self.compute_disparity()
            
        except Exception as e:
            self.get_logger().error(f"Error processing left image: {e}")

    def right_cb(self, msg):
        """Callback for right rectified image"""
        try:
            with self.lock:
                self.right_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
                self.right_stamp = msg.header.stamp
                
            # Try to compute disparity if both images are available
            self.compute_disparity()
            
        except Exception as e:
            self.get_logger().error(f"Error processing right image: {e}")

    def compute_disparity(self):
        """Compute disparity map from synchronized left and right images"""
        with self.lock:
            if self.left_image is None or self.right_image is None:
                return
            
            # Check if images are synchronized (same timestamp or close)
            if self.left_stamp and self.right_stamp:
                time_diff = abs(
                    (self.left_stamp.sec + self.left_stamp.nanosec * 1e-9) -
                    (self.right_stamp.sec + self.right_stamp.nanosec * 1e-9)
                )
                if time_diff > 0.1:  # 100ms threshold
                    self.get_logger().warn(f"Images not synchronized: {time_diff}s difference")
                    return
            
            left_img = self.left_image.copy()
            right_img = self.right_image.copy()
        
        try:
            # Compute disparity map
            disparity = self.stereo.compute(left_img, right_img)
            
            # Disparity is returned as int16 with 16x scale factor
            # Convert to float32 for visualization and further processing
            disparity_float = disparity.astype(np.float32) / 16.0
            
            # Apply filtering to remove outliers
            disparity_filtered = self.post_process_disparity(disparity_float)
            
            # Normalize for visualization (0-255 range)
            disparity_display = np.clip(disparity_filtered / disparity_filtered.max() * 255, 0, 255)
            disparity_display = disparity_display.astype(np.uint8)
            
            # Convert to ROS Image message
            disparity_msg = self.bridge.cv2_to_imgmsg(disparity_display, encoding="mono8")
            disparity_msg.header = self.left_image if isinstance(self.left_image, dict) else None
            # If header from message is available, use that
            disparity_msg.header.stamp = self.left_stamp if self.left_stamp else self.get_clock().now().to_msg()
            disparity_msg.header.frame_id = "stereo_frame"
            
            self.pub.publish(disparity_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error computing disparity: {e}")

    def post_process_disparity(self, disparity):
        """Post-process disparity map to remove outliers and fill holes"""
        # Remove invalid disparities (typically 0 or very small values)
        disparity_filtered = disparity.copy()
        
        # Median filter to smooth and remove noise
        disparity_filtered = cv2.medianBlur((disparity_filtered * 16).astype(np.uint16), 5)
        disparity_filtered = disparity_filtered.astype(np.float32) / 16.0
        
        # Remove negative disparities
        disparity_filtered[disparity_filtered < 0] = 0
        
        return disparity_filtered


def main():
    rclpy.init()
    node = DisparityNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
