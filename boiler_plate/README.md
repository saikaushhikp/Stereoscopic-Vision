<!-- # Stereoscopic Vision Pipeline

A complete ROS2-based stereoscopic vision system for real-time depth estimation and 3D reconstruction. This pipeline processes stereo images from two synchronized cameras to generate dense depth maps and 3D point clouds suitable for robotics, AR/VR, and autonomous navigation applications. -->

## Flow

```
Raw Images → Rectification → Disparity → Depth → PointCloud
    ↓            ↓              ↓          ↓          ↓
 (Camera)   (Calibrated)   (StereoSGBM)  (Formula)  (3D Points)
```

## Structure

```
<repo>/boiler_plate/
├── src/
│   ├── stereo_bringup/
│   │   ├── launch/
│   │   │   └── stereo_pipeline_launch.py    # Main launch file
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   ├── stereo_calibration/
│   │   ├── stereo_calibrate.py              # Calibration script
│   │   ├── config/
│   │   │   └── stereo.yaml                  # Calibration parameters
│   │   └── package.xml
│   │
│   ├── stereo_perception/
│   │   ├── nodes/
│   │   │   ├── rectification_node.py        # Image rectification
│   │   │   ├── disparity_node.py            # Disparity computation
│   │   │   └── depth_node.py                # Depth conversion
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   └── stereo_mapping/
│       ├── nodes/
│       │   └── pointcloud_node.py           # 3D point cloud generation
│       └── package.xml
└── README.md
```

## initial requisites (ASSUNING, NOT YET CONFIRMED)

### Hardware
- Stereo camera pair (e.g., Zed, RealSense, USB stereo cameras)
- Calibration pattern (Charuco board or checkerboard)

### Software
- ROS2 (Humble or later)
- Python 3.8+
- OpenCV 4.5+
- NumPy
- PyYAML

### Installation

- Key requirements and setup have been listed in `requirements.txt` and `setup.py` files.
- These are intented to guide via installation and setup, however, it is advisable to manually install dependencies to ensure compatibility.

```bash
# Install dependencies
sudo apt-get update
sudo apt-get install python3-opencv python3-numpy python3-pyyaml
sudo apt-get install python3-launch python3-launch-ros

# Clone and build
cd ~/Kezual/stereo/Stereoscopic-Vision/boiler_plate
```


## 1: Camera Calibration

Before running the pipeline, calibrate your stereo cameras to obtain intrinsic and extrinsic parameters.

### Generate Calibration Pattern

```python
import cv2

# Generate Charuco board (5x7, 25mm squares)
arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
board = cv2.aruco.CharucoBoard((5, 7), 0.025, 0.02, arucoDict)
img = cv2.aruco.generateImage(board, (2000, 2000))
cv2.imwrite('charuco_board.png', img)

# Print at high resolution (recommended 50mm+ squares for clarity)
```

### Run Calibration

```bash
# Create directories for calibration images
mkdir -p calib_data/left calib_data/right

# Capture calibration images:
# 1. Left camera images -> calib_data/left/ (left_*.png)
# 2. Right camera images -> calib_data/right/ (right_*.png)
# Capture 15-20 images from different angles and distances

# Run calibration script
python3 src/stereo_calibration/stereo_calibrate.py calib_data/left calib_data/right --output src/stereo_calibration/config/stereo.yaml --pattern charuco --board-width 5 --board-height 7 --square-size 0.025
```

**Output:**
```yaml
K_left: [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
D_left: [k1, k2, p1, p2, k3]
K_right: [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
D_right: [k1, k2, p1, p2, k3]
R: [rotation_matrix]
T: [translation_vector]
baseline: 0.12
focal_length: 700.0
```

## 2: Configure Camera Input

Set up your stereo camera to publish to:
- `/left/image_raw` - Left camera raw images
- `/right/image_raw` - Right camera raw images

**Example with USB cameras:**
```bash
ros2 launch usb_camera camera.launch.py camera_name:=left video_device:=/dev/video0

ros2 launch usb_camera camera.launch.py camera_name:=right video_device:=/dev/video1
```

## 3: Run the Pipeline
```bash
# Basic launch
ros2 launch stereo_bringup stereo_pipeline_launch.py

# With custom calibration file
ros2 launch stereo_bringup stereo_pipeline_launch.py calibration_file:=/path/to/stereo.yaml

# With simulation time (Gazebo)
ros2 launch stereo_bringup stereo_pipeline_launch.py use_sim_time:=true
```

## Node Details

### Rectification Node
- **Package**: `stereo_perception`
- **Executable**: `rectification_node`
- **Subscriptions**:
  - `/left/image_raw` (sensor_msgs/Image)
  - `/right/image_raw` (sensor_msgs/Image)
- **Publications**:
  - `/left/image_rect` (sensor_msgs/Image)
  - `/right/image_rect` (sensor_msgs/Image)
- **Description**: Undistorts and rectifies images using calibration parameters

### Disparity Node
- **Package**: `stereo_perception`
- **Executable**: `disparity_node`
- **Subscriptions**:
  - `/left/image_rect` (sensor_msgs/Image)
  - `/right/image_rect` (sensor_msgs/Image)
- **Publications**:
  - `/stereo/disparity` (sensor_msgs/Image, mono8)
- **Algorithm**: StereoSGBM (Configurable to StereoBM for speed)
- **Description**: Computes pixel-wise disparity between rectified images

### Depth Node
- **Package**: `stereo_perception`
- **Executable**: `depth_node`
- **Subscriptions**:
  - `/stereo/disparity` (sensor_msgs/Image)
- **Publications**:
  - `/stereo/depth` (sensor_msgs/Image, mono16, in mm)
- **Formula**: $\text{depth} = \frac{f \times b}{d}$
  - $f$ = focal length (pixels)
  - $b$ = baseline (meters)
  - $d$ = disparity (pixels)
- **Description**: Converts disparity maps to metric depth using camera parameters

### PointCloud Node
- **Package**: `stereo_mapping`
- **Executable**: `pointcloud_node`
- **Subscriptions**:
  - `/stereo/depth` (sensor_msgs/Image)
- **Publications**:
  - `/stereo/points` (sensor_msgs/PointCloud2)
- **Description**: Back-projects depth map to 3D point cloud using camera intrinsics

## Visualization

### RViz Visualization

```bash
ros2 run rviz2 rviz2

# Add displays:
# 1. Image - Subscribe to /left/image_rect (rectified images)
# 2. Image - Subscribe to /stereo/disparity (disparity visualization)
# 3. Image - Subscribe to /stereo/depth (depth visualization)
# 4. PointCloud2 - Subscribe to /stereo/points (3D point cloud)
```

### Check Topics

```bash
# List all active topics
ros2 topic list

# Visualize message types
ros2 topic info /stereo/points

# Echo data from specific topic
ros2 topic echo /stereo/disparity --once
```

## Parameters & Tuning

### Disparity Node Parameters
- `numDisparities`: Search range (80-256 pixels) - higher = larger depth range
- `blockSize`: Window size (5-15) - larger = smoother but slower
- `P1`, `P2`: Smoothness constraints - control depth smoothness
- `speckleWindowSize`: Post-processing parameter - remove noise
- `speckleRange`: Maximum variation in speckle region

### Depth Node Parameters
- Load `focal_length` and `baseline` from `stereo.yaml`
- Depth range clipping: 0.1m - 10m (configurable)
- Output: uint16 in millimeters

<!-- ## Performance Notes

| Component | Latency | Notes |
|-----------|---------|-------|
| Rectification | ~5-10ms | Real-time with pre-computed maps |
| Disparity (StereoSGBM) | ~50-100ms | Better quality, slower |
| Disparity (StereoBM) | ~20-30ms | Faster, lower quality |
| Depth Conversion | ~2-5ms | Lightweight |
| PointCloud Gen | ~10-20ms | Depends on cloud density |
| **Total Pipeline** | **~80-150ms** | Varies with image resolution |

**Optimization Tips:**
- Reduce image resolution for faster processing
- Switch to StereoBM for real-time applications
- Adjust `numDisparities` to match baseline distance
- Enable GPU acceleration if available

## Troubleshooting

### Issue: Images not synchronized
- **Solution**: Ensure both cameras publish with synchronized timestamps
- Check: `ros2 topic echo /left/image_raw --once` and compare timestamps

### Issue: Poor depth quality
- **Solution**: Recalibrate cameras with more diverse calibration images
- Ensure adequate lighting and texture in the scene
- Increase `numDisparities` for better accuracy

### Issue: Calibration fails
- **Solution**: Ensure clear Charuco/checkerboard detection
- Use at least 15 calibration image pairs
- Vary board position, distance, and angle

### Issue: Large disparity processing latency
- **Solution**: Reduce image resolution
- Use StereoBM instead of StereoSGBM
- Lower `numDisparities` value -->

## Configuration Files

### stereo.yaml
Contains camera calibration parameters:
- Camera matrices (K_left, K_right)
- Distortion coefficients (D_left, D_right)
- Rotation and translation between cameras (R, T)
- Baseline distance and focal length

Update after calibration:
```bash
python3 src/stereo_calibration/stereo_calibrate.py calib_data/left calib_data/right --output src/stereo_calibration/config/stereo.yaml
```

<!-- ## Expected Outputs

- [x] **Rectified Images**: Epipolar aligned, artifact-free  
- [x] **Disparity Maps**: Pixel-wise depth difference visualization  
- [x] **Depth Maps**: Metric depth in meters (mono16, mm scale)  
- [x] **Point Cloud**: Dense 3D reconstruction (PointCloud2 format) -->


## References

- OpenCV Stereo Vision: https://docs.opencv.org/master/d9/d0c/group__calib3d.html
- ROS2 Image Processing: https://github.com/ros-perception/vision_opencv
- StereoSGBM Algorithm: Hirschmuller, H. (2008)
