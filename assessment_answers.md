# AIG240 Project 1 - Assessment Questions

## Question 1: What command did you use to create a ROS package?

I used `catkin_create_pkg` to create the package:

```bash
catkin_create_pkg lab3_turtlesim rospy std_msgs geometry_msgs
```

This creates a package with the dependencies I need for Python and ROS messages.

---

## Question 2: Explain why and how you used ROS messages in your program.

I used the `Twist` message from `geometry_msgs` to send velocity commands to the turtle.

```python
from geometry_msgs.msg import Twist

twist = Twist()
twist.linear.x = 2.0   # forward speed
twist.angular.z = 1.0  # turn speed
publisher.publish(twist)
```

The turtle listens on `/turtle1/cmd_vel` topic for these messages and moves accordingly.

---

## Question 3: Describe the steps to launch ROS, TurtleSim, and your ROS node simultaneously.

Using the launch file:

```bash
roslaunch lab3_turtlesim turtle_controller.launch
```

Or manually with 3 terminals:

1. Terminal 1: `roscore`
2. Terminal 2: `rosrun turtlesim turtlesim_node`
3. Terminal 3: `rosrun lab3_turtlesim turtle_controller.py turtle1`

---

## Question 4: How do you verify that your ROS node is publishing messages correctly?

I use `rostopic echo` to see the messages:

```bash
rostopic echo /turtle1/cmd_vel
```

This shows the velocity messages being sent to the turtle in real-time.
