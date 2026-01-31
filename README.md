# AIG240 Project 1 - ROS1 TurtleSim Controller

Seneca Polytechnic - AIG240 Robotics

## Overview

A ROS1 keyboard controller for TurtleSim that supports **multi-key simultaneous input** for diagonal/curved movement paths.

## Features

- **Multi-key Support**: Press `w + a` together for forward-left curve
- **Contradicting Key Handling**: Pressing `w + s` cancels out (no movement)
- **Instant Stop**: Turtle stops immediately when keys are released
- **Diagonal Shortcuts**: `q`, `e`, `z`, `c` for quick diagonal movement

## Controls

| Key(s) | Action |
|--------|--------|
| `w` | Forward |
| `s` | Backward |
| `a` | Turn Left |
| `d` | Turn Right |
| `w + a` | Forward + Left (curve) |
| `w + d` | Forward + Right (curve) |
| `q` | Forward-Left shortcut |
| `e` | Forward-Right shortcut |
| `z` | Backward-Left shortcut |
| `c` | Backward-Right shortcut |
| `ESC` | Exit controller |

## Installation

### Prerequisites

- ROS1 (Melodic/Noetic)
- Python 2.7+ or 3.x
- pynput library

### Setup

```bash
# 1. Install pynput
pip install pynput

# 2. Clone this repository to your catkin workspace
cd ~/catkin_ws/src
git clone https://github.com/kevinhust/aig240project1.git
cd aig240project1

# 3. Copy the ROS package
cp -r lab3_turtlesim ~/catkin_ws/src/

# 4. Build
cd ~/catkin_ws
catkin_make

# 5. Source the workspace
source ~/catkin_ws/devel/setup.bash
```

Or use the setup script:

```bash
chmod +x setup.sh
./setup.sh
```

## Usage

### Method 1: Launch File (Recommended)

```bash
roslaunch lab3_turtlesim turtle_controller.launch
```

### Method 2: Manual (3 Terminals)

```bash
# Terminal 1
roscore

# Terminal 2
rosrun turtlesim turtlesim_node

# Terminal 3
rosrun lab3_turtlesim turtle_controller.py turtle1
```

## Project Structure

```
aig240project1/
├── README.md
├── assessment_answers.md      # Answers to project questions
├── setup.sh                   # Quick setup script
└── lab3_turtlesim/           # ROS Package
    ├── CMakeLists.txt
    ├── package.xml
    ├── launch/
    │   └── turtle_controller.launch
    └── scripts/
        └── turtle_controller.py
```

## Assessment Questions

See [assessment_answers.md](assessment_answers.md) for detailed answers to:

1. What command did you use to create a ROS package?
2. Explain why and how you used ROS messages in your program.
3. Describe the steps to launch ROS, TurtleSim, and your ROS node simultaneously.
4. How do you verify that your ROS node is publishing messages correctly?

## License

MIT
