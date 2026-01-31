#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ROS1 TurtleSim Keyboard Controller
Controls turtle movement using keyboard input (w/a/s/d and q/e/z/c)
Compatible with Python 2.7 (ROS Melodic)

AIG240 Project 1 - Single Key Mode (Easy Version)
- q/e/z/c provide diagonal movement shortcuts
- Turtle STOPS when key is released (meets project requirement)
"""

import rospy
import sys
import termios
import tty
import select
from geometry_msgs.msg import Twist

# Key bindings: (linear_velocity, angular_velocity)
KEY_BINDINGS = {
    'w': (1.0, 0.0),    # Forward
    's': (-1.0, 0.0),   # Backward
    'a': (0.0, 1.0),    # Turn left
    'd': (0.0, -1.0),   # Turn right
    'q': (1.0, 1.0),    # Forward + left (circular path)
    'e': (1.0, -1.0),   # Forward + right (circular path)
    'z': (-1.0, 1.0),   # Backward + left
    'c': (-1.0, -1.0),  # Backward + right
}

# Movement speeds
LINEAR_SPEED = 2.0
ANGULAR_SPEED = 2.0


class TurtleController:
    """Keyboard controller for TurtleSim using standard Python libraries"""
    
    def __init__(self, turtle_name):
        self.turtle_name = turtle_name
        
        # Initialize ROS node
        rospy.init_node('turtle_controller', anonymous=True)
        
        # Create publisher for velocity commands
        topic_name = '/{}/cmd_vel'.format(turtle_name)
        self.pub = rospy.Publisher(topic_name, Twist, queue_size=10)
        
        # Store original terminal settings
        self.old_settings = termios.tcgetattr(sys.stdin)
        
        rospy.loginfo("=" * 50)
        rospy.loginfo("Turtle Controller started for: {}".format(turtle_name))
        rospy.loginfo("=" * 50)
        rospy.loginfo("Controls:")
        rospy.loginfo("  w = forward    s = backward")
        rospy.loginfo("  a = turn left  d = turn right")
        rospy.loginfo("  q = forward-left   e = forward-right")
        rospy.loginfo("  z = backward-left  c = backward-right")
        rospy.loginfo("  Ctrl+C = exit")
        rospy.loginfo("=" * 50)
    
    def get_key(self, timeout=0.1):
        """
        Get a single keypress with timeout.
        Returns empty string if no key pressed within timeout.
        """
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        return key
    
    def create_twist_msg(self, linear, angular):
        """Create a Twist message with given velocities"""
        twist = Twist()
        twist.linear.x = linear * LINEAR_SPEED
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular * ANGULAR_SPEED
        return twist
    
    def stop_turtle(self):
        """Stop the turtle by publishing zero velocity"""
        stop_msg = self.create_twist_msg(0.0, 0.0)
        self.pub.publish(stop_msg)
    
    def run(self):
        """Main control loop"""
        rate = rospy.Rate(20)  # 20 Hz
        
        try:
            while not rospy.is_shutdown():
                key = self.get_key()
                
                if key == '\x03':  # Ctrl+C
                    break
                
                if key in KEY_BINDINGS:
                    linear, angular = KEY_BINDINGS[key]
                    twist_msg = self.create_twist_msg(linear, angular)
                    self.pub.publish(twist_msg)
                    rospy.loginfo("Key: {} -> linear={:.1f}, angular={:.1f}".format(
                        key, twist_msg.linear.x, twist_msg.angular.z))
                else:
                    # No valid key pressed - STOP the turtle
                    # This ensures turtle stops when key is released
                    self.stop_turtle()
                
                rate.sleep()
                
        except Exception as e:
            rospy.logerr("Error: {}".format(e))
        finally:
            # Ensure turtle stops when exiting
            self.stop_turtle()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            rospy.loginfo("Turtle Controller stopped")


def print_usage():
    """Print usage instructions"""
    print("")
    print("Usage: rosrun lab3_turtlesim turtle_controller.py <turtle_name>")
    print("")
    print("Example:")
    print("  rosrun lab3_turtlesim turtle_controller.py turtle1")
    print("")


def main():
    # Check command line arguments
    if len(sys.argv) < 2:
        print_usage()
        sys.exit(1)
    
    turtle_name = sys.argv[1]
    
    try:
        controller = TurtleController(turtle_name)
        controller.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
