#!/usr/bin/python3

# Software License Agreement (MIT License)
#
# Copyright (c) 2025, Iran University of Science and Technology
# Robot Programming with ROS2 Course
# All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# Author: Erfan Riazati
# Contact: erf.riazati@gmail.com
#
# Robot Programming with ROS2 | Fall 2025
# Iran University of Science and Technology
#
# Week 1
# Assignment:
#  
# Custom Keyboard Teleoperation for Turtlesim
#
# This node reads keyboard inputs and publishes velocity commands
# of type geometry_msgs/msg/Twist to the /turtle1/cmd_vel topic in order to
# control the motion of a Turtlesim turtle.

import rclpy
from rclpy.node import Node

# >>>>>>>>>>> STUDENT IMPLEMENTATION >>>>>>>>>>>
#
# TODO: Import all necessary Python modules for ROS 2 functionality,
# including the ROS client library (rclpy) and the message types
# required for your node, such as Twist from geometry_msgs.msg.
#
from geometry_msgs.msg import Twist
# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

import sys, termios, tty, select

class TurtleKeyboardController(Node):
    def __init__(self):
        super().__init__('turtle_keyboard_controller')

        # >>>>>>>>>>> STUDENT IMPLEMENTATION >>>>>>>>>>>
        #
        # TODO: Create a publisher to the '/turtle1/cmd_vel' topic to 
        # generate the turtle's linear and angular velocities. Use the 
        # 'Twist' message type from 'geometry_msgs.msg'. The callback function
        # that processes incoming messages should be named 'pose_callback'.
        #
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

        # Define fixed turtle velocities
        self.linear_speed = 1.5
        self.angular_speed = 1.57

        self.get_logger().info("Keyboard control started. Use keys: I (,), J, L, and K.")
        self.get_logger().info("I: forward | ,: backward | J: turn left | L: turn right | K: stop | Ctrl+C: exit")

        self.run()

    def get_key(self):
        """Read a single key press (non-blocking)."""
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def run(self):
        """Main control loop."""

        # >>>>>>>>>>> STUDENT IMPLEMENTATION >>>>>>>>>>>
        #
        # TODO: Create an instance of the Twist message type named
        # 'twist'. This message will store the linear and angular velocity
        # commands that correspond to each keyboard input (e.g., I, ,,
        # J, L, K) and will be published to control the turtle's motion.
        # 
        twist = Twist()

        # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

        try:
            while rclpy.ok():
                key = self.get_key().lower()

                if key == 'i':  # Forward
                    twist.linear.x = self.linear_speed
                    twist.angular.z = 0.0
                elif key == ',':  # Backward
                    twist.linear.x = -self.linear_speed
                    twist.angular.z = 0.0
                elif key == 'j':  # Turn left
                    twist.linear.x = 0.0
                    twist.angular.z = self.angular_speed
                elif key == 'l':  # Turn right
                    twist.linear.x = 0.0
                    twist.angular.z = -self.angular_speed
                elif key == 'k':  # Stop
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                elif key == '\x03':  # Ctrl+C
                    break
                else:
                    # no valid key, keep last twist
                    pass

                # >>>>>>>>>>> STUDENT IMPLEMENTATION >>>>>>>>>>>
                #
                # TODO: Publish the 'twist' message to the '/turtle1/cmd_vel' topic
                # 
                self.cmd_pub.publish(twist)
                # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

        except KeyboardInterrupt:
            pass
        finally:
            # >>>>>>>>>>> STUDENT IMPLEMENTATION >>>>>>>>>>>
            #
            # TODO: Set both linear and angular velocities of the
            # 'twist' message to zero and publish it to the '/turtle1/cmd_vel'
            # topic. This ensures that the turtle stops moving safely when
            # the program ends.
            # 
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            try:
                self.cmd_pub.publish(twist)
            except Exception:
                pass

            # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

            self.get_logger().info("Exiting keyboard controller...")


def main(args=None):
    

    # >>>>>>>>>>> STUDENT IMPLEMENTATION >>>>>>>>>>>
    #
    # TODO: Initialize the ROS 2 Python client library (rclpy) and create
    # an instance of the TurtleTargetPlotter node. This will prepare the
    # node for spinning and allow it to publish velocity commands.
    #
    rclpy.init(args=args)
    node = None
    try:
        node = TurtleKeyboardController()
        # TurtleKeyboardController.__init__ calls run() and blocks until exit.
        # After it returns, just continue to cleanup below.
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        rclpy.shutdown()


    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>


if __name__ == '__main__':
    main()
