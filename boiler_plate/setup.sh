#!/usr/bin/env bash

set -e

echo "========================================="
echo " Setting up ROS 2 + Python environment"
echo "========================================="

# -----------------------------
# 1. Conda Environment
# -----------------------------
ENV_NAME=stereo_env

echo "[1/6] Creating Conda environment: $ENV_NAME"

conda create -y -n $ENV_NAME python=3.10
conda activate $ENV_NAME

# -----------------------------
# 2. System Dependencies
# -----------------------------
echo "[2/6] Installing system dependencies"

sudo apt update
sudo apt install -y curl gnupg lsb-release build-essential cmake git

# -----------------------------
# 3. Install ROS 2 Humble
# -----------------------------
echo "[3/6] Installing ROS 2 Humble"

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-ros-base ros-humble-launch ros-humble-launch-ros ros-humble-sensor-msgs ros-humble-std-msgs ros-humble-cv-bridge ros-humble-image-transport python3-colcon-common-extensions python3-rosdep

# -----------------------------
# 4. ROS Environment Setup
# -----------------------------
echo "[4/6] Initializing rosdep"

sudo rosdep init || true
rosdep update

# Auto-source ROS in shell
if ! grep -q "ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

source /opt/ros/humble/setup.bash

# -----------------------------
# 5. Python Dependencies
# -----------------------------
echo "[5/6] Installing Python dependencies"

pip install --upgrade pip
pip install numpy opencv-python pyyaml

# -----------------------------
# 6. Verify Installation
# -----------------------------
echo "[6/6] Verifying imports"

python - <<EOF
import cv2
import numpy
import yaml
import rclpy
from cv_bridge import CvBridge
print("All core dependencies imported successfully")
EOF

echo "========================================="
echo " Setup complete!"
echo " Activate with: conda activate $ENV_NAME"
echo "========================================="
