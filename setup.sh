#!/bin/bash
# ============================================
# AIG240 Project 1 - Quick Setup Script
# ROS1 TurtleSim Multi-Key Controller
# ============================================

set -e

echo "======================================"
echo "AIG240 Project 1 - Setup"
echo "======================================"

# Check if ROS is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "Error: ROS is not sourced."
    echo "Please run: source /opt/ros/<distro>/setup.bash"
    exit 1
fi

echo "[1/5] Checking catkin workspace..."
CATKIN_WS=~/catkin_ws

if [ ! -d "$CATKIN_WS/src" ]; then
    echo "Creating catkin workspace..."
    mkdir -p $CATKIN_WS/src
    cd $CATKIN_WS
    catkin_make
fi

echo "[2/5] Installing pynput..."
pip install pynput 2>/dev/null || pip3 install pynput 2>/dev/null || echo "Warning: Could not install pynput automatically"

echo "[3/5] Copying ROS package..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_SRC="$SCRIPT_DIR/lab3_turtlesim"
PACKAGE_DST="$CATKIN_WS/src/lab3_turtlesim"

if [ -d "$PACKAGE_DST" ]; then
    echo "Removing old package..."
    rm -rf $PACKAGE_DST
fi

cp -r $PACKAGE_SRC $PACKAGE_DST
chmod +x $PACKAGE_DST/scripts/*

echo "[4/5] Building workspace..."
cd $CATKIN_WS
catkin_make

echo "[5/5] Sourcing workspace..."
source $CATKIN_WS/devel/setup.bash

# Add to bashrc if not already present
if ! grep -q "source ~/catkin_ws/devel/setup.bash" ~/.bashrc; then
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    echo "Added workspace to ~/.bashrc"
fi

echo ""
echo "======================================"
echo "Setup Complete!"
echo "======================================"
echo ""
echo "To run the controller:"
echo "  roslaunch lab3_turtlesim turtle_controller.launch"
echo ""
echo "Controls: w/a/s/d (can combine!), q/e/z/c, ESC to exit"
echo "======================================"
