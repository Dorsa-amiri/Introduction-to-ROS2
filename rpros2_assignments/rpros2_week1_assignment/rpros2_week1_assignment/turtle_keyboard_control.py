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
# Turtle Pose Plotter and Random Target Generator
#
# This node subscribes to /turtle1/pose of type turtlesim/msg/Pose
# and visualizes the turtle position on a 10x10 grid using matplotlib. A 1x1
# target square is randomly generated. When the turtle reaches the square,
# a new target is generated and displayed. The node provides a real-time visual
# representation of the turtle navigating towards successive random targets.


import matplotlib.pyplot as plt
import numpy as np
import random

import rclpy
from rclpy.node import Node

# >>>>>>>>>>> STUDENT IMPLEMENTATION >>>>>>>>>>>
#
# TODO: Import all necessary Python modules for ROS 2 functionality,
# including the ROS client library (rclpy) and the message types
# required for your node, such as Pose from turtlesim.msg.
#
from turtlesim.msg import Pose
# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

class TurtleTargetPlotter(Node):
    def __init__(self):
        super().__init__('turtle_target_plotter')

        # >>>>>>>>>>> STUDENT IMPLEMENTATION >>>>>>>>>>>
        #
        # TODO: Implement the following functionalities:
        #
        # 1. Create a subscription to the '/turtle1/pose' topic to receive
        #    the turtle's current position and orientation. Use the 
        #    'Pose' message type from 'turtlesim.msg'. The callback function
        #    that processes incoming messages should be named 'pose_callback'.
        #
        # 2. Create a timer that periodically updates the plot to reflect
        #    the turtle's current position and the target square. The timer
        #    should call the 'update_plot' function at a fixed interval.
        #
        # Ensure that both the subscriber and timer are correctly initialized
        # within the node so the plot updates dynamically as the turtle moves.
        #

        # create subscription to /turtle1/pose
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )

        # create a timer to refresh the plot
        self.timer = self.create_timer(0.1, self.update_plot)

        # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

        # Turtle state
        self.x = 5.0
        self.y = 5.0
        self.theta = 0.0

        # Target (1x1 square)
        self.target_x = random.uniform(0, 9)
        self.target_y = random.uniform(0, 9)
        self.target_size = 1.0

        # Matplotlib setup
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(6, 6))
        self.setup_plot()

        self.get_logger().info("Turtle target plotter started.")

    def setup_plot(self):
        """Initialize 10x10 grid."""
        self.ax.set_xlim(0, 11)
        self.ax.set_ylim(0, 11)
        self.ax.set_xticks(np.arange(0, 11, 1))
        self.ax.set_yticks(np.arange(0, 11, 1))
        self.ax.grid(True)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_title('Turtle1 Pose with Random Target')

    def pose_callback(self, msg):
        """Update turtle pose."""

        # >>>>>>>>>>> STUDENT IMPLEMENTATION >>>>>>>>>>>
        #
        # TODO: Update the turtle's current state variables (self.x, self.y,
        # and self.theta) using the data received from the Pose message of
        # the subscribed '/turtle1/pose' topic.
        #

        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta

        # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

        # Check if turtle is inside the target square
        if (self.target_x <= self.x <= self.target_x + self.target_size) and \
           (self.target_y <= self.y <= self.target_y + self.target_size):
            self.get_logger().info("Target reached! Generating new target...")
            self.generate_new_target()

    def generate_new_target(self):
        """Generate a new random target within the grid."""
        self.target_x = random.uniform(0, 9)
        self.target_y = random.uniform(0, 9)

    def update_plot(self):
        """Redraw plot with current turtle pose and target."""
        self.ax.cla()
        self.setup_plot()

        # Draw the target square
        target_rect = plt.Rectangle(
            (self.target_x, self.target_y),
            self.target_size,
            self.target_size,
            color='green',
            alpha=0.4,
            label='Target'
        )
        self.ax.add_patch(target_rect)

        # Draw turtle arrow
        arrow_scale = 0.5
        dx = np.cos(self.theta) * arrow_scale
        dy = np.sin(self.theta) * arrow_scale
        self.ax.arrow(
            self.x, self.y, dx, dy,
            head_width=0.2, head_length=0.25,
            fc='blue', ec='blue', linewidth=2
        )

        # Draw turtle position
        self.ax.plot(self.x, self.y, 'ro', label='Turtle')

        plt.draw()
        plt.pause(0.001)


def main(args=None):

    # >>>>>>>>>>> STUDENT IMPLEMENTATION >>>>>>>>>>>
    #
    # TODO: Initialize the ROS 2 Python client library (rclpy) and create
    # an instance of the TurtleTargetPlotter node. This will prepare the
    # node for spinning and allow it to subscribe to the turtle's pose
    # and update the plot dynamically.
    #
    rclpy.init(args=args)
    node = TurtleTargetPlotter()

    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        plt.ioff()
        plt.show()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
