#!/bin/bash

# F1TENTH Jetson Environment Setup Script
# This script sets up the virtual environment and installs dependencies for the F1TENTH RL project.

set -e

PROJECT_ROOT=$(pwd)
VENV_NAME="jetson-ros2"
VENV_PATH="$PROJECT_ROOT/$VENV_NAME"

echo "==== Starting F1TENTH Jetson Setup ===="

# 1. Update and Install System Dependencies
echo "[1/5] Installing system dependencies..."
sudo apt-get update
sudo apt-get install -y python3-venv python3-pip python3-dev \
    i2c-tools libi2c-dev python3-setuptools

# 2. Virtual Environment Setup
if [ ! -d "$VENV_PATH" ]; then
    echo "[2/5] Creating virtual environment: $VENV_NAME..."
    python3 -m venv "$VENV_PATH" --system-site-packages
else
    echo "[2/5] Virtual environment $VENV_NAME already exists."
fi

# 3. Activate and Install Python Packages
echo "[3/5] Installing Python packages... (This may take a while)"
source "$VENV_PATH/bin/activate"

# Upgrade pip
pip install --upgrade pip

# Install common libraries
pip install numpy stable-baselines3 shimmy gym gymnasium

# Install Adafruit libraries for PCA9685
pip install Adafruit-Blinka adafruit-circuitpython-pca9685

# Note: PyTorch on Jetson usually needs a specific wheel from NVIDIA.
# We check if torch is already there (from system-site-packages) or install a generic one.
if python3 -c "import torch; print(torch.__version__)" &> /dev/null; then
    echo "PyTorch is already installed."
else
    echo "PyTorch not found. Installing generic version (Note: NVIDIA-optimized version recommended)..."
    pip install torch torchvision torchaudio
fi

# 4. ROS 2 Workspace Build
echo "[4/5] Building ROS 2 workspace..."
cd "$PROJECT_ROOT/ros2_ws"
# Clean previous builds to ensure consistency
rm -rf build/ install/ log/

# Source ROS 2 (Humble) - assuming it's in standard location
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# Build with the venv python
colcon build --packages-select f1tenth_rl

# 5. User Group Setup
echo "[5/5] Checking user groups for I2C access..."
if groups $USER | grep &>/dev/null "\bi2c\b"; then
    echo "User is already in the i2c group."
else
    echo "Adding user to the i2c group..."
    sudo usermod -aG i2c $USER
    echo "NOTE: You may need to logout and login for group changes to take effect."
fi

echo "==== Setup Complete! ===="
echo "To start working, run:"
echo "source $VENV_NAME/bin/activate"
echo "source ros2_ws/install/setup.bash"
echo "ros2 launch f1tenth_rl f1tenth_rl.launch.py"
