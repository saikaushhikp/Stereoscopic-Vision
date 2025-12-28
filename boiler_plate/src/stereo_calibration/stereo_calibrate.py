"""
Offline stereo calibration script.
Produces intrinsic + extrinsic parameters using Charuco or checkerboard patterns.
"""

import cv2
import numpy as np
import yaml
import os
from pathlib import Path


def calibrate_stereo_cameras(left_images_dir, right_images_dir, output_yaml, 
                             pattern_type='charuco', board_size=(5, 7), square_size=0.025):
    """
    Calibrate stereo cameras using Charuco or checkerboard patterns.
    
    Args:
        left_images_dir: Directory containing left camera calibration images
        right_images_dir: Directory containing right camera calibration images
        output_yaml: Path to save calibration parameters
        pattern_type: 'charuco' or 'checkerboard'
        board_size: Board dimensions (width, height)
        square_size: Size of each square in meters
    """
    
    # Create ArUco dictionary and board
    if pattern_type == 'charuco':
        arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        board = cv2.aruco.CharucoBoard(board_size, square_size, square_size * 0.8, arucoDict)
        detectorParams = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(arucoDict, detectorParams)
    
    # Get calibration image paths
    left_images = sorted(Path(left_images_dir).glob('*.png')) + sorted(Path(left_images_dir).glob('*.jpg'))
    right_images = sorted(Path(right_images_dir).glob('*.png')) + sorted(Path(right_images_dir).glob('*.jpg'))
    
    if len(left_images) == 0 or len(right_images) == 0:
        print(f"Error: No images found. Left: {len(left_images)}, Right: {len(right_images)}")
        return False
    
    print(f"Found {len(left_images)} left images and {len(right_images)} right images")
    
    objpoints = []  # 3D points in real world space
    imgpoints_left = []  # 2D points in left image plane
    imgpoints_right = []  # 2D points in right image plane
    
    # Generate object points (0, 0, 0), (square_size, 0, 0), ...
    objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2)
    objp *= square_size
    
    for left_img_path, right_img_path in zip(left_images, right_images):
        left_img = cv2.imread(str(left_img_path))
        right_img = cv2.imread(str(right_img_path))
        
        if left_img is None or right_img is None:
            print(f"Warning: Could not load {left_img_path} or {right_img_path}")
            continue
        
        gray_left = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)
        
        if pattern_type == 'charuco':
            # Detect Charuco corners
            corners_left, ids_left, rejected_left = detector.detectMarkers(gray_left)
            corners_right, ids_right, rejected_right = detector.detectMarkers(gray_right)
            
            if ids_left is not None and ids_right is not None:
                ret_left, charuco_corners_left, charuco_ids_left = cv2.aruco.interpolateCornersCharuco(
                    corners_left, ids_left, gray_left, board)
                ret_right, charuco_corners_right, charuco_ids_right = cv2.aruco.interpolateCornersCharuco(
                    corners_right, ids_right, gray_right, board)
                
                if ret_left and ret_right and len(charuco_corners_left) > 3 and len(charuco_corners_right) > 3:
                    objpoints.append(objp[:len(charuco_corners_left)])
                    imgpoints_left.append(charuco_corners_left)
                    imgpoints_right.append(charuco_corners_right)
                    print(f"Successfully detected pattern in {left_img_path.name}")
        else:
            # Checkerboard pattern
            ret_left, corners_left = cv2.findChessboardCorners(gray_left, board_size, None)
            ret_right, corners_right = cv2.findChessboardCorners(gray_right, board_size, None)
            
            if ret_left and ret_right:
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners_left = cv2.cornerSubPix(gray_left, corners_left, (11, 11), (-1, -1), criteria)
                corners_right = cv2.cornerSubPix(gray_right, corners_right, (11, 11), (-1, -1), criteria)
                
                objpoints.append(objp)
                imgpoints_left.append(corners_left)
                imgpoints_right.append(corners_right)
                print(f"Successfully detected pattern in {left_img_path.name}")
    
    if len(objpoints) < 3:
        print("Error: Not enough valid calibration images found")
        return False
    
    print(f"\nCalibrating using {len(objpoints)} image pairs...")
    
    # Calibrate individual cameras
    ret_left, K_left, D_left, _, _ = cv2.calibrateCamera(
        objpoints, imgpoints_left, gray_left.shape[::-1], None, None)
    ret_right, K_right, D_right, _, _ = cv2.calibrateCamera(
        objpoints, imgpoints_right, gray_right.shape[::-1], None, None)
    
    # Stereo calibration
    ret, K_left, D_left, K_right, D_right, R, T, E, F = cv2.stereoCalibrate(
        objpoints, imgpoints_left, imgpoints_right,
        K_left, D_left, K_right, D_right,
        gray_left.shape[::-1],
        criteria=(cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 100, 1e-5),
        flags=cv2.CALIB_FIX_INTRINSIC)
    
    if not ret:
        print("Stereo calibration failed")
        return False
    
    # Calculate baseline and focal length
    baseline = np.linalg.norm(T)
    focal_length = K_left[0, 0]
    
    print(f"Calibration successful!")
    print(f"Baseline: {baseline:.4f} m")
    print(f"Focal length: {focal_length:.2f} pixels")
    print(f"Left reprojection error: {ret_left:.4f}")
    print(f"Right reprojection error: {ret_right:.4f}")
    
    # Save to YAML
    calibration_data = {
        'K_left': K_left.tolist(),
        'D_left': D_left.flatten().tolist(),
        'K_right': K_right.tolist(),
        'D_right': D_right.flatten().tolist(),
        'R': R.tolist(),
        'T': T.tolist(),
        'baseline': float(baseline),
        'focal_length': float(focal_length)
    }
    
    os.makedirs(os.path.dirname(output_yaml), exist_ok=True)
    with open(output_yaml, 'w') as f:
        yaml.dump(calibration_data, f, default_flow_style=False)
    
    print(f"Calibration saved to {output_yaml}")
    return True


def main():
    """
    Main function for stereo calibration.
    
    Usage:
        python stereo_calibrate.py <left_images_dir> <right_images_dir> [output_yaml]
    
    Example:
        python stereo_calibrate.py ./calib_left ./calib_right ./config/stereo.yaml
    """
    import argparse
    
    parser = argparse.ArgumentParser(description='Stereo camera calibration')
    parser.add_argument('left_dir', help='Directory containing left camera calibration images')
    parser.add_argument('right_dir', help='Directory containing right camera calibration images')
    parser.add_argument('--output', '-o', default='stereo.yaml', help='Output YAML file')
    parser.add_argument('--pattern', '-p', default='charuco', choices=['charuco', 'checkerboard'],
                        help='Calibration pattern type')
    parser.add_argument('--board-width', type=int, default=5, help='Board width (number of squares)')
    parser.add_argument('--board-height', type=int, default=7, help='Board height (number of squares)')
    parser.add_argument('--square-size', type=float, default=0.025, help='Square size in meters')
    
    args = parser.parse_args()
    
    calibrate_stereo_cameras(
        args.left_dir, args.right_dir, args.output,
        pattern_type=args.pattern,
        board_size=(args.board_width, args.board_height),
        square_size=args.square_size
    )


if __name__ == "__main__":
    main()
