#!/usr/bin/env python3

import sys

import rclpy
from rclpy.node import Node

# >>>>>>>>>>> STUDENT IMPLEMENTATION >>>>>>>>>>>
#
# TODO: Import your custom service interface defined
# in the rpros2_interfaces/srv directory. This service
# will be used to convert robot linear and angular
# velocities (v, w) into individual wheel speeds (v_l, v_r).
#
from rpros2_interfaces_clean.srv import Wheel2RobotVelocity
# >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

class Wheel2RobotVelocityClient(Node):
    def __init__(self):
        super().__init__('wheel2robot_velocity_client')
        
        # >>>>>>>>>>> STUDENT IMPLEMENTATION >>>>>>>>>>>
        #
        # TODO: Create a ROS 2 service client for the service
        # named 'compute_wheel_velocities' using your custom
        # interface (rpros2_interfaces.srv/Wheel2RobotVelocity).
        #
        # Implement a timeout loop to wait until the service
        # becomes available before proceeding.
        #
        # Also, create a request object instance that will hold
        # the request part of the service message (v and w values).
        #
        self.client = self.create_client(Wheel2RobotVelocity, 'compute_wheel_velocities')

        timeout_sec = 1.0
        self.get_logger().info('Waiting for service compute_wheel_velocities...')
        while not self.client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().info('Service not available yet, waiting...')
        # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

    def send_request(self, v, w):
        # >>>>>>>>>>> STUDENT IMPLEMENTATION >>>>>>>>>>>
        #
        # TODO: Assign the input arguments (v, w) to the request
        # fields of your service message.
        #
        # Use the asynchronous service call method This allows 
        # the node to continue running while waiting for the 
        # service response.
        #
        req = Wheel2RobotVelocity.Request()
        req.v = float(v)
        req.w = float(w)
        future = self.client.call_async(req)
        return future
        # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

def main(args=None):
    rclpy.init(args=args)
    
    client = Wheel2RobotVelocityClient()

    # >>>>>>>>>>> STUDENT IMPLEMENTATION >>>>>>>>>>>
    #
    # TODO: Retrieve the linear (v) and angular (w) velocity
    # values from the command line arguments using `sys.argv[]`.
    #
    # Send the service request to calculate individual wheel
    # velocities and spin the node to process the async future.
    #
    # Once the response is received, print the results (v_l, v_r),
    # then destroy the node to stop execution.
    #
    if len(sys.argv) < 3:
        client.get_logger().error('Usage: ros2 run rpros2_week2_assignment wheel2robot_vel_client <v> <w>')
        client.destroy_node()
        rclpy.shutdown()
        return

    v = float(sys.argv[1])
    w = float(sys.argv[2])

    future = client.send_request(v, w)
    rclpy.spin_until_future_complete(client, future)
    if future.result() is not None:
        res = future.result()
        client.get_logger().info(f"Response: v_l={res.v_l:.6f}, v_r={res.v_r:.6f}")
        print(f"{res.v_l} {res.v_r}")
    else:
        client.get_logger().error('Service call failed')
    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
