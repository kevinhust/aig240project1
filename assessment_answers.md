# AIG240 Project 1 - Assessment Questions Answers

## Question 1: What command did you use to create a ROS package?

To create a ROS package, I used the `catkin_create_pkg` command:

```bash
catkin_create_pkg lab3_turtlesim rospy std_msgs geometry_msgs
```

This command creates a new ROS package named `lab3_turtlesim` with the following dependencies:
- **rospy**: Python client library for ROS
- **std_msgs**: Standard ROS message types
- **geometry_msgs**: Geometric primitive messages (including Twist for velocity commands)

---

## Question 2: Explain why and how you used ROS messages in your program.

### Why I used ROS messages:
ROS messages are the standard way for nodes to communicate in a ROS system. In this project, I needed to send velocity commands to control the turtle in TurtleSim. The turtle subscribes to velocity commands on its `cmd_vel` topic.

### How I used ROS messages:
I used the **Twist** message type from the `geometry_msgs` package:

```python
from geometry_msgs.msg import Twist

# Create a Twist message
twist = Twist()
twist.linear.x = 2.0   # Linear velocity (forward/backward)
twist.angular.z = 1.0  # Angular velocity (rotation)

# Publish the message to control the turtle
publisher.publish(twist)
```

The Twist message contains:
- `linear` (Vector3): x, y, z components for linear velocity
- `angular` (Vector3): x, y, z components for angular velocity

For TurtleSim, only `linear.x` (forward/backward) and `angular.z` (rotation) are used since it moves in a 2D plane.

---

## Question 3: Describe the steps to launch ROS, TurtleSim, and your ROS node simultaneously.

### Method 1: Using Launch File (Recommended)
```bash
# Single command to start everything
roslaunch lab3_turtlesim turtle_controller.launch
```

### Method 2: Manual Steps (Three Terminals)

**Terminal 1 - Start ROS Master:**
```bash
roscore
```

**Terminal 2 - Start TurtleSim:**
```bash
rosrun turtlesim turtlesim_node
```

**Terminal 3 - Start the Controller:**
```bash
rosrun lab3_turtlesim turtle_controller.py turtle1
```

---

## Question 4: How do you verify that your ROS node is publishing messages correctly?

There are several methods to verify message publishing:

### Method 1: Use `rostopic echo`
```bash
# Monitor messages on the turtle's velocity topic
rostopic echo /turtle1/cmd_vel
```
This shows real-time messages being published to the topic.

### Method 2: Use `rostopic hz`
```bash
# Check publishing frequency
rostopic hz /turtle1/cmd_vel
```
This shows how often messages are being published.

### Method 3: Use `rostopic list`
```bash
# List all active topics
rostopic list
```
Verify that `/turtle1/cmd_vel` appears in the list.

### Method 4: Use `rqt_graph`
```bash
rqt_graph
```
This displays a visual graph showing node connections and message flow.

### Method 5: Check `rosnode info`
```bash
rosnode info /turtle_controller
```
This shows what topics the node is publishing to and subscribing from.
