#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ROS1 TurtleSim Multi-Key Controller
Supports simultaneous key presses (e.g., w+a for forward-left)
AIG240 Project 1

Requires: pip install pynput
"""

import rospy
import sys
import threading
from geometry_msgs.msg import Twist

try:
    from pynput import keyboard
except ImportError:
    print("=" * 50)
    print("Error: pynput not installed!")
    print("Please run: pip install pynput")
    print("=" * 50)
    sys.exit(1)

# Movement speeds
LINEAR_SPEED = 2.0
ANGULAR_SPEED = 2.0


class TurtleController:
    """
    Multi-key keyboard controller for TurtleSim.
    
    Supports simultaneous key presses:
    - w + a = forward + left turn (circular path)
    - Contradicting keys (w + s) cancel out
    """
    
    def __init__(self, turtle_name):
        self.turtle_name = turtle_name
        
        # Track currently pressed keys
        self.pressed_keys = set()
        self.lock = threading.Lock()
        self.running = True
        
        # Initialize ROS node
        rospy.init_node('turtle_controller', anonymous=True)
        
        # Create publisher for velocity commands
        topic_name = '/{}/cmd_vel'.format(turtle_name)
        self.pub = rospy.Publisher(topic_name, Twist, queue_size=10)
        
        rospy.loginfo("=" * 50)
        rospy.loginfo("Turtle Controller started for: {}".format(turtle_name))
        rospy.loginfo("=" * 50)
        rospy.loginfo("Controls:")
        rospy.loginfo("  w = forward    s = backward")
        rospy.loginfo("  a = turn left  d = turn right")
        rospy.loginfo("  Combine keys: w+a = forward-left curve")
        rospy.loginfo("  q/e/z/c = diagonal shortcuts")
        rospy.loginfo("  ESC = exit")
        rospy.loginfo("=" * 50)
    
    def on_press(self, key):
        """Handle key press events"""
        try:
            k = key.char.lower()
            with self.lock:
                self.pressed_keys.add(k)
        except AttributeError:
            # Special key (ESC, etc.)
            if key == keyboard.Key.esc:
                self.running = False
                return False  # Stop listener
    
    def on_release(self, key):
        """Handle key release events"""
        try:
            k = key.char.lower()
            with self.lock:
                self.pressed_keys.discard(k)
        except AttributeError:
            pass
    
    def calculate_velocity(self):
        """
        Calculate velocity based on currently pressed keys.
        Contradicting keys cancel each other out.
        """
        linear = 0.0
        angular = 0.0
        
        with self.lock:
            keys = self.pressed_keys.copy()
        
        # Handle diagonal shortcut keys first
        if 'q' in keys:
            return 1.0, 1.0  # forward + left
        elif 'e' in keys:
            return 1.0, -1.0  # forward + right
        elif 'z' in keys:
            return -1.0, 1.0  # backward + left
        elif 'c' in keys:
            return -1.0, -1.0  # backward + right
        
        # Handle w/s for linear movement (contradicting = no movement)
        if 'w' in keys and 's' in keys:
            linear = 0.0  # Contradicting - no movement
        elif 'w' in keys:
            linear = 1.0
        elif 's' in keys:
            linear = -1.0
        
        # Handle a/d for angular movement (contradicting = no rotation)
        if 'a' in keys and 'd' in keys:
            angular = 0.0  # Contradicting - no rotation
        elif 'a' in keys:
            angular = 1.0
        elif 'd' in keys:
            angular = -1.0
        
        return linear, angular
    
    def run(self):
        """Main control loop"""
        # Start keyboard listener in separate thread
        listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        listener.start()
        
        rate = rospy.Rate(20)  # 20 Hz
        
        try:
            while not rospy.is_shutdown() and self.running and listener.running:
                linear, angular = self.calculate_velocity()
                
                # Create and publish Twist message
                twist = Twist()
                twist.linear.x = linear * LINEAR_SPEED
                twist.linear.y = 0.0
                twist.linear.z = 0.0
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = angular * ANGULAR_SPEED
                
                self.pub.publish(twist)
                
                # Log movement (throttled to avoid spam)
                if linear != 0 or angular != 0:
                    rospy.loginfo_throttle(0.5, "Moving: linear={:.1f}, angular={:.1f}".format(
                        twist.linear.x, twist.angular.z))
                
                rate.sleep()
                
        except Exception as e:
            rospy.logerr("Error: {}".format(e))
        finally:
            # Ensure turtle stops when exiting
            self.pub.publish(Twist())
            listener.stop()
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
