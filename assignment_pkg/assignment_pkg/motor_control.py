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
GOAL_PRINT_1 = "\n\n GOAL RECEIVED:\n\n  X = "
GOAL_PRINT_2 = "  Y = "
GOAL_PRINT_3 = "\n ROBOT IS ALLIGNING WITH THE CAMERA "
GOAL_PRINT_4 = "\n ROBOT IS REACHING THE GOAL POSITION "
GOAL_PRINT_5 = "\n GOAL REACHED\n"

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
            Float64,
            'camera_theta_goal',
            self.theta_callback,
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
        # PUBBLISHER TO ROTATE THE CAMERA IN THE OPPOSITE ROTATION
        self.publisher_rotation = self.create_publisher(Float64, 'inverse_rotation', 10)
        # FLAG VARIABLE
        self.theta = 0.0
        self.theta_goal = 0.0
        self.flag = 0
        # Counter for doesn't allow the node to block inside the callback of the action server (recursive function)
        self.dt = 0.1
        self.control_loop_timer = self.create_timer(self.dt, self.robot_movement)


    def robot_movement(self):

        if self.flag == 0:
            # ricevo costantemente il mio theta da /odom e aspetto che mi venga inviato
            # il theta goal da /camera_theta_goal
            time.sleep(self.dt/2)
            print(" ASPETTO IL THETA BOIA CAGNA \n")

        elif self.flag == 1:
            # allineo il robot con il marker
            self.allign_camera()
            print(" MI ALLINEO CON LA CAMERA DI MERDA \n")

        elif self.flag == 2:
            # raggiungo il marker
            self.go(1)
            print(" CHE CAZZO FACCIO ORA? VADO DRITTO FIGA \n")
            time.sleep(self.dt/2)

        elif self.flag == 3:
            # vado indietro un pelo
            # Go backwards to allow the next turn
            print(" BOIA LADRA TORNO INDIETRO \n\n")
            self.go(-1)
            time.sleep(self.dt * 5)
            self.stop()
            self.flag = 0

    def odom_callback(self, msg):
        # Callback for processing odometry data
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        # Convert quaternion to theta
        self.theta = math.atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)
        if self.theta < 0:
            self.theta = math.pi + (math.pi + self.theta)

    def theta_callback(self, msg):
        # Callback for processing odometry data
        if self.flag == 0:
            self.theta_goal = msg.data
            self.flag += 1

    def reached_callback(self, msg):
        # Callback for processing odometry data
        print(" \n\n\n\n                    REACHED CALLBACK ")
        stop = msg.data
        if stop == True and self.flag == 2:
            print(" I'm in")
            self.stop()
            self.flag += 1

    def allign_camera(self):
        msg = Float64()
        # Align the camera with the goal orientation
        # Next line is to handle the case where the angle wraps around from the maximum value (6.28) to the minimum value (0.0)
        diff = (self.theta_goal - self.theta + 6.28) % 6.28
        if diff > 3.14: # Half of the maximum range
            msg.data = -0.25
            self.rotate(-1)
        else:
            msg.data = 0.25
            self.rotate(1)
        if abs(self.theta_goal - self.theta) < 0.02:
            #print(GOAL_PRINT_4)
            msg.data = 0.0
            self.stop()
            self.flag += 1
        self.publisher_rotation.publish(msg)

    def stop(self):
        # Stop the robot
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.publisher_.publish(cmd_vel)
        time.sleep(0.1)

    def go(self, sign):
        # Stop the robot
        cmd_vel = Twist()
        cmd_vel.linear.x = sign * MAX_VEL
        cmd_vel.angular.z = 0.0
        self.publisher_.publish(cmd_vel)

    def rotate(self, sign):
        # Rotate the robot in place
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = (sign * MAX_VEL)/2
        self.publisher_.publish(cmd_vel)

"""
    def wait_for_input(self):
        while True:
            user_input = input("Quando vuoi andare avanti: ")
            if user_input:
                break
"""

def main(args=None):
    rclpy.init(args=args)

    try:
        motor_control = MotorControl()

        # Create a multi-threaded executor
        executor = MultiThreadedExecutor()

        # Add the node to the executor
        executor.add_node(motor_control)

        try:
            # Use spin_once() instead of spin() to allow for multi-threaded execution
            while rclpy.ok():
                executor.spin_once()

        finally:
            # Clean up
            rclpy.shutdown()

    except Exception as e:
        print(f"Error in main: {str(e)}")

if __name__ == '__main__':
    main()

