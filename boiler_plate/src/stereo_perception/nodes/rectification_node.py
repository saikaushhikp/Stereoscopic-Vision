import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import yaml
import os


class RectificationNode(Node):
    def __init__(self):
        super().__init__('rectification_node')
        
        self.bridge = CvBridge()
        self.calib_params = None
        self.left_map1 = None
        self.left_map2 = None
        self.right_map1 = None
        self.right_map2 = None
        
        # Load calibration parameters
        config_dir = os.path.join(
            os.path.dirname(__file__), '../../stereo_calibration/config'
        )
        calib_file = os.path.join(config_dir, 'stereo.yaml')
        
        if os.path.exists(calib_file):
            with open(calib_file, 'r') as f:
                self.calib_params = yaml.safe_load(f)
            self.initialize_rectification()
            self.get_logger().info("Calibration loaded and rectification maps computed")
        else:
            self.get_logger().warn(f"Calibration file not found at {calib_file}")
        
        self.sub_left = self.create_subscription(
            Image, '/left/image_raw', self.left_cb, 10)

        self.sub_right = self.create_subscription(
            Image, '/right/image_raw', self.right_cb, 10)

        self.pub_left = self.create_publisher(
            Image, '/left/image_rect', 10)

        self.pub_right = self.create_publisher(
            Image, '/right/image_rect', 10)

        self.get_logger().info("Rectification node started")

    def initialize_rectification(self):
        """Compute rectification maps from calibration parameters"""
        if not self.calib_params:
            self.get_logger().error("No calibration parameters loaded")
            return
        
        K_left = np.array(self.calib_params['K_left'])
        D_left = np.array(self.calib_params['D_left'])
        K_right = np.array(self.calib_params['K_right'])
        D_right = np.array(self.calib_params['D_right'])
        R = np.array(self.calib_params['R'])
        T = np.array(self.calib_params['T'])
        
        # Stereo rectify
        R1, R2, P1, P2, Q, validRoi1, validRoi2 = cv2.stereoRectify(
            K_left, D_left, K_right, D_right,
            (640, 480),  # Image size (should be read from actual image)
            R, T,
            flags=cv2.CALIB_ZERO_DISPARITY,
            alpha=0.0
        )
        
        # Compute rectification maps
        self.left_map1, self.left_map2 = cv2.initUndistortRectifyMap(
            K_left, D_left, R1, P1, (640, 480), cv2.CV_32F)
        self.right_map1, self.right_map2 = cv2.initUndistortRectifyMap(
            K_right, D_right, R2, P2, (640, 480), cv2.CV_32F)

    def left_cb(self, msg):
        """Rectify left image"""
        if self.left_map1 is None:
            return
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Update map dimensions if image size differs
            if cv_image.shape[1] != self.left_map1.shape[1] or cv_image.shape[0] != self.left_map1.shape[0]:
                self.reinitialize_maps(cv_image.shape)
            
            # Apply rectification
            rectified = cv2.remap(cv_image, self.left_map1, self.left_map2, cv2.INTER_LINEAR)
            
            # Publish rectified image
            rectified_msg = self.bridge.cv2_to_imgmsg(rectified, encoding="bgr8")
            rectified_msg.header = msg.header
            self.pub_left.publish(rectified_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error processing left image: {e}")

    def right_cb(self, msg):
        """Rectify right image"""
        if self.right_map1 is None:
            return
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Apply rectification
            rectified = cv2.remap(cv_image, self.right_map1, self.right_map2, cv2.INTER_LINEAR)
            
            # Publish rectified image
            rectified_msg = self.bridge.cv2_to_imgmsg(rectified, encoding="bgr8")
            rectified_msg.header = msg.header
            self.pub_right.publish(rectified_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error processing right image: {e}")

    def reinitialize_maps(self, image_shape):
        """Reinitialize rectification maps for new image size"""
        if not self.calib_params:
            return
        
        height, width = image_shape[:2]
        K_left = np.array(self.calib_params['K_left'])
        D_left = np.array(self.calib_params['D_left'])
        K_right = np.array(self.calib_params['K_right'])
        D_right = np.array(self.calib_params['D_right'])
        R = np.array(self.calib_params['R'])
        T = np.array(self.calib_params['T'])
        
        R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
            K_left, D_left, K_right, D_right,
            (width, height), R, T,
            flags=cv2.CALIB_ZERO_DISPARITY,
            alpha=0.0
        )
        
        self.left_map1, self.left_map2 = cv2.initUndistortRectifyMap(
            K_left, D_left, R1, P1, (width, height), cv2.CV_32F)
        self.right_map1, self.right_map2 = cv2.initUndistortRectifyMap(
            K_right, D_right, R2, P2, (width, height), cv2.CV_32F)


def main():
    rclpy.init()
    node = RectificationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
