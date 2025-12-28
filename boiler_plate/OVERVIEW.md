# Implementation Guide

Quick reference for deploying the stereoscopic vision system on a robotic platform.

## System Architecture

<!-- ### Data Flow

```
Sensors → Perception → Mapping → Navigation
   ↓           ↓          ↓         ↓
 Stereo    Disparity   SLAM/   Path Planning
+ IMU      + Depth     Mapping  + Avoidance
``` -->

### Layer Breakdown

| Layer | Node | Input | Output |
|-------|------|-------|--------|
| **Acquisition** | Camera Driver | Raw Images | `sensor_msgs/Image` |
| **Perception** | Rectification | Raw Images | Rectified Images |
| | Disparity | Rect. Images | Disparity Map |
| | Depth | Disparity | Depth Map (mono16) |
| **Mapping** | PointCloud | Depth Map | `PointCloud2` |


###  Hardware Setup
- Mount stereo camera on robot
- Install JetPack 6.x + ROS2 Humble
- **Calibrate cameras** $\to$ stereo.yaml
- Verify TF tree: `base_link` $\to$ `camera_link`

### Perception 
- Configure disparity node (StereoSGBM)
- Tune disparity range (baseline-dependent)
- Test in various lighting/texture conditions

### Localization + Mapping
- Validate depth map quality
- Build 3D point clouds
- Check for artifacts in textureless zones

<!-- ### Navigation
- Integrate with Nav2
- Test obstacle avoidance
- Autonomous mission validation -->

## Hardware Requirements(ASSUMING, NOT YET CONFIRMED)

**Compute:**
- Jetson Orin Nano (8GB) or AGX Orin

**Camera:**
- Stereolabs ZED 2i (outdoor, long-range)
- Intel RealSense D435i (indoor, close-range)

**Calibration:**
- Large CharuCo board (A3+ size)

**OS:**
- Ubuntu 22.04 LTS
- ROS2 Humble
- OpenCV 4.5+, NumPy, PyYAML

## Key Tuning Parameters

### Disparity Node
- `numDisparities`: 80-256 pixels (larger = bigger depth range)
- `blockSize`: 5-15 (larger = smoother but slower)
- `speckleWindowSize`: 50-150 (filter noise)

### Depth Node
- Baseline: From calibration
- Focal length: From calibration
- Clipping range: 0.1m - 10m

## Common Issues & Fixes( From popular "stack-exchange" questions hehehe)

| Problem | Cause | Fix |
|---------|-------|-----|
| Poor depth in textureless zones | Low feature density | Enable IR projector or increase blockSize |
| Motion blur | Slow shutter speed | Reduce exposure time < 10ms |
| System lag | High CPU load | Lower `numDisparities` or switch to StereoBM |
| Calibration fails | Blurry/unclear pattern | Use larger Charuco board (50mm+ squares) |
| Temperature issues | Jetson running hot | Enable active cooling: `jetson_clocks` |


## Verifications to be done

- [ ] Stereo cameras synchronized (same timestamp)
- [ ] Calibration error < 0.1 pixels
- [ ] Rectified images are epipolar-aligned
- [ ] Disparity maps smooth (no speckles)
- [ ] Depth values in expected range (0.1-10m)
- [ ] PointCloud density > 50% valid points
- [ ] Nav2 responding to obstacles in real-time

## Next to-dos

1. **Calibrate** cameras first (most critical)
2. **Test perception** pipeline independently
3. **Integrate** with Nav2 after validation
4. **Iterate** on parameters based on environment
