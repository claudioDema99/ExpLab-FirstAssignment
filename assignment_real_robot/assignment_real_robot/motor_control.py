#!/usr/bin/env python3
import rclpy
import math
import time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64, Bool
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

MAX_VEL = 0.5
class MotorControl(Node):

    def __init__(self):
        super().__init__('motor_control')
        # SUBSCRIBER TO ODOM
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.subscription
        # SUBSCRIBER TO CAMERA_THETA_GOAL
        self.subscription = self.create_subscription(
            Bool,
            'camera_on_off',
            self.camera_callback,
            10)
        self.subscription
        # SUBSCRIBER TO MARKER_REACHED
        self.subscription = self.create_subscription(
            Bool,
            'marker_reached',
            self.reached_callback,
            10)
        self.subscription
        # PUBLISHER TO CMD_VEL
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # PUBBLISHER TO INVERSE_ROTATION
        self.publisher_rotation = self.create_publisher(Float64, 'inverse_rotation', 10)
        # FLAG VARIABLE
        self.theta = 0.0
        self.theta_goal = 0.0
        self.flag = 0
        self.dt = 0.1
        self.reached_marker = 0
        # TIMER
        self.control_loop_timer = self.create_timer(self.dt, self.robot_movement)

    def robot_movement(self):
        # Main control loop

        if self.flag == 0:
            # wait for theta
            self.rotate(1)
            self.get_logger().info('Rotating for theta...')

        elif self.flag == 1:
            # go straight
            self.go(1)
            self.get_logger().info('Going straight...')

        elif self.flag == 2:
            # go back and stop
            self.go(-1)
            time.sleep(self.dt)
            self.go(-1)
            time.sleep(self.dt)
            self.stop()
            time.sleep(self.dt * 2)
            self.flag = 0
            self.reached_marker += 1
            self.get_logger().info('Going back...')

        if self.reached_marker == 4:
            # shutdown the node when the robot has reached the 4 markers
            self.get_logger().info('Shutting down...')
            self.destroy_node()
            rclpy.shutdown()

    def odom_callback(self, msg):
        # callback for updating the robot orientation
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        # convert quaternion to theta
        self.theta = math.atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)
        if self.theta < 0:
            self.theta = math.pi + (math.pi + self.theta)

    def camera_callback(self, msg):
        # callback for updating the goal orientation
        mode = msg.data
        if mode == False and self.flag == 0:
            self.stop()
            time.sleep(self.dt * 2)
            self.flag += 1
            time.sleep(self.dt * 2)

    def reached_callback(self, msg):
        # callback for stopping the robot when the marker is reached
        stop = msg.data
        if stop == True and self.flag == 1:
            self.stop()
            time.sleep(self.dt * 2)
            self.flag += 1
            time.sleep(self.dt * 2)
        elif stop == False and self.flag == 1:
            self.stop()
            self.flag = 0

    def stop(self):
        # stop the robot
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.publisher_.publish(cmd_vel)
        time.sleep(0.1)

    def go(self, sign):
        # move the robot forward or backward
        cmd_vel = Twist()
        cmd_vel.linear.x = (sign * MAX_VEL) * 0.50
        cmd_vel.angular.z = 0.0
        self.publisher_.publish(cmd_vel)

    def rotate(self, sign):
        # rotate the robot clockwise or counterclockwise
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = sign * MAX_VEL
        self.publisher_.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)

    try:
        motor_control = MotorControl()

        # create a multi-threaded executor
        executor = MultiThreadedExecutor()

        # add the node to the executor
        executor.add_node(motor_control)

        try:
            # use spin_once() instead of spin() to allow for multi-threaded execution
            while rclpy.ok():
                executor.spin_once()

        finally:
            # clean up
            rclpy.shutdown()

    except Exception as e:
        print(f"Error in main: {str(e)}")

if __name__ == '__main__':
    main()

