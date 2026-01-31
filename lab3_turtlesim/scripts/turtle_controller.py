#!/usr/bin/env python
# Turtle Controller - AIG240 Project 1
# Controls turtle in TurtleSim using keyboard

import rospy
import sys
import termios
import tty
import select
from geometry_msgs.msg import Twist

# Key mappings: (linear, angular)
KEYS = {
    'w': (1.0, 0.0),   # forward
    's': (-1.0, 0.0),  # backward
    'a': (0.0, 1.0),   # left
    'd': (0.0, -1.0),  # right
    'q': (1.0, 1.0),   # forward-left
    'e': (1.0, -1.0),  # forward-right
    'z': (-1.0, 1.0),  # backward-left
    'c': (-1.0, -1.0), # backward-right
}

SPEED = 2.0

def get_key(old_settings):
    """Read a key from keyboard"""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    return key

def main():
    if len(sys.argv) < 2:
        print("Usage: rosrun lab3_turtlesim turtle_controller.py <turtle_name>")
        sys.exit(1)
    
    turtle_name = sys.argv[1]
    
    rospy.init_node('turtle_controller')
    pub = rospy.Publisher('/' + turtle_name + '/cmd_vel', Twist, queue_size=10)
    
    old_settings = termios.tcgetattr(sys.stdin)
    
    print("Turtle Controller for " + turtle_name)
    print("w/a/s/d = move, q/e/z/c = diagonal, Ctrl+C = exit")
    
    rate = rospy.Rate(20)
    
    try:
        while not rospy.is_shutdown():
            key = get_key(old_settings)
            
            if key == '\x03':  # Ctrl+C
                break
            
            twist = Twist()
            
            if key in KEYS:
                twist.linear.x = KEYS[key][0] * SPEED
                twist.angular.z = KEYS[key][1] * SPEED
            
            pub.publish(twist)
            rate.sleep()
            
    finally:
        # stop turtle
        pub.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

if __name__ == '__main__':
    main()
