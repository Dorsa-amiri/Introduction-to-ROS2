import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'rpros2_week1_assignment'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # >>>>>>>>>>> STUDENT IMPLEMENTATION >>>>>>>>>>>
        #
        # TODO: Make sure to integrate your custom launch file in the 'data_files'
        # section of 'setup.py'. This ensures that the launch file is installed
        # properly and can be accessed using ROS 2 launch commands.
        #
        ('share/rpros2_week1_assignment/launch', ['launch/turtle_target_teleop.launch.py']),

        # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='erfan',
    maintainer_email='erf.riazati@gmail.com',
    description='Robot Programming with ROS2 Assignments | Week 1 | ROS Publishers/Subscribers, Packages, and Launch Files',
    license='MIT',
    entry_points={
        'console_scripts': [
            # >>>>>>>>>>> STUDENT IMPLEMENTATION >>>>>>>>>>>
            #
            # TODO: Add the 'turtle_keyboard_control' and 'turtle_target_plot' nodes
            # to the package entry points in the 'console_scripts' section of
            # 'setup.py'. This ensures that the nodes can be launched and run
            # using 'ros2 run' commands.
            #
            'turtle_keyboard_control = rpros2_week1_assignment.turtle_keyboard_control:main',
            'turtle_target_plot = rpros2_week1_assignment.turtle_target_plot:main',
            # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        ],
    },
)
