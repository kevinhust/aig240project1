# AIG240 Project 1 - ROS1 TurtleSim Controller

Seneca Polytechnic - AIG240 Robotics

## Overview

A ROS1 keyboard controller for TurtleSim with diagonal movement support using `q/e/z/c` keys.

**Compatible with**: ROS Melodic (Python 2.7) and ROS Noetic (Python 3)

## Features

- **Keyboard Control**: Use `w/a/s/d` for basic movement
- **Diagonal Movement**: `q/e/z/c` for curved paths (forward-left, forward-right, etc.)
- **Instant Stop**: Turtle stops immediately when keys are released
- **No External Dependencies**: Uses only standard Python libraries

## Controls

| Key | Action |
|-----|--------|
| `w` | Forward |
| `s` | Backward |
| `a` | Turn Left |
| `d` | Turn Right |
| `q` | Forward + Left (curve) |
| `e` | Forward + Right (curve) |
| `z` | Backward + Left |
| `c` | Backward + Right |
| `Ctrl+C` | Exit |

## Installation

### Quick Setup

```bash
# 1. Clone repository
cd ~
git clone https://github.com/kevinhust/aig240project1.git
cd aig240project1

# 2. Run setup script
chmod +x setup.sh
./setup.sh
```

### Manual Setup

```bash
# 1. Copy package to catkin workspace
cp -r lab3_turtlesim ~/catkin_ws/src/

# 2. Make script executable
chmod +x ~/catkin_ws/src/lab3_turtlesim/scripts/turtle_controller

# 3. Build
cd ~/catkin_ws
catkin_make

# 4. Source workspace
source ~/catkin_ws/devel/setup.bash
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
rosrun lab3_turtlesim turtle_controller turtle1
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
        ├── spawn_turtle      # Script to spawn extra turtles
        └── turtle_controller # Main controller script
```

## Assessment Questions

See [assessment_answers.md](assessment_answers.md) for answers to:

1. What command did you use to create a ROS package?
2. Explain why and how you used ROS messages in your program.
3. Describe the steps to launch ROS, TurtleSim, and your ROS node simultaneously.
4. How do you verify that your ROS node is publishing messages correctly?

## License

MIT
