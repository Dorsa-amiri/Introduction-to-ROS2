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
# Launch File for Turtle Teleoperation and Visualization
#
# This launch file starts the turtlesim node, launches the TurtleTargetPlotter
# node that visualizes the turtle position and random target squares, and opens a
# separate terminal window to run the TurtleKeyboardController node for interactive
# keyboard teleoperation. The launch ensures all three components run together for
# a seamless Week 1 assignment demonstration.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([

        # >>>>>>>>>>> STUDENT IMPLEMENTATION >>>>>>>>>>>
        #
        # TODO: Add the 'turtlesim_node' and 'turtle_target_plot' nodes to this
        # launch file. The 'turtle_keyboard_control' node is already configured
        # to run in a separate xterm window. 

        # Note: If 'xterm' is not installed on your system, install it using:
        #       sudo apt install xterm
        #
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),

        Node(
            package='rpros2_week1_assignment',
            executable='turtle_target_plot',
            name='turtle_target_plot',
            output='screen'
        ),
        
        # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        
        ExecuteProcess(
            cmd=['xterm', '-e', 'ros2 run rpros2_week1_assignment turtle_keyboard_control'],
            output='screen'
        )
    ])
