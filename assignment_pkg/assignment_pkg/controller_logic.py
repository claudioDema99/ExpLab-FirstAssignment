#!/usr/bin/env python3
import rclpy
import math
import numpy as np
import time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer
from action_msgs.msg import GoalStatus
from custom_action_interfaces_1explab.action import MarkerPosition
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from simple_pid import PID

MAX_VEL = 0.5
MAX_ANG_VEL = 0.4
GOAL_PRINT_1 = "\n\n GOAL RECEIVED:\n\n  X = "
GOAL_PRINT_2 = "  Y = "
GOAL_PRINT_3 = "\n ROBOT IS ALLIGNING WITH THE CAMERA "
GOAL_PRINT_4 = "\n ROBOT IS REACHING THE GOAL POSITION "
GOAL_PRINT_5 = "\n GOAL REACHED\n"

class ControllerLogic(Node):

    def __init__(self):
        super().__init__('controller_logic')
        # Create an action server for the MarkerPosition custom action
        self.action_server = ActionServer(self, MarkerPosition, 'marker_position', self.execute_callback)
        # SUBSCRIBER TO ODOM
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.subscription
        # PUBLISHER TO CMD_VEL
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # INTERNAL STATE VARIABLES
        self.x = 0
        self.y = 0
        self.theta = 0
        # GOAL VARIABLES
        self.x_goal = 0.0
        self.y_goal = 0.0
        self.theta_marker = 0.0
        self.theta_d = 0.0
        self.x_d = 0.0
        self.y_d = 0.0
        # FLAG VARIABLE
        self.flag = 0
        self.linear_pid = PID(Kp=1.0, Ki=0.1, Kd=0.01, setpoint=0)
        self.angular_pid = PID(Kp=1.0, Ki=0.1, Kd=0.01, setpoint=0)
        # Counter for doesn't allow the node to block inside the callback of the action server (recursive function)
        self.i = 0

    def execute_callback(self, goal_handle):
        # Callback when a new goal is received
        goal = goal_handle.request
        # transformation matrix in order to transform the relative position of the target (wrt the robot frame) to the absolute position
        t_m = self.transformation_matrix(self.x, self.y, self.theta)
        x = np.array([goal.x_goal, goal.y_goal, 1])
        x_t = np.dot(t_m, x)
        self.x_goal = x_t[0]
        self.y_goal = x_t[1]
        self.theta_marker = goal.theta_goal

        self.calculate_desired_target()
        #print(GOAL_PRINT_1 + str(self.x_goal))
        #print(GOAL_PRINT_2 + str(self.y_goal))
        goal_handle.succeed()
        # Start the robot movement logic
        #print(GOAL_PRINT_3)
        self.robot_movement()
        if self.flag == 7:
            print(self.i)
            self.flag = 0
            self.i = 0
            # If the robot reached the goal, send a success result to the action client
            result = MarkerPosition.Result()
            result.reached = True
            GoalStatus.STATUS_SUCCEEDED
            return result
        # da mettere
        if self.flag == 8:
            print(self.i)
            self.flag = 0
            self.i = 0
            # If the robot reached the goal, send a success result to the action client
            result = MarkerPosition.Result()
            result.reached = False
            GoalStatus.STATUS_ABORTED
            return result

    def robot_movement(self):
        # Logic for robot movement: CAZZATA NON È PIU COSIIIIII
        # First of all, I allign the robot with the camera 
        # (theta_gol is the angle of the camera when it has detected the marker)
        # So it begins the effective recursive function 
        # that stops only if the robot is close to the goal position
        cmd_vel = Twist()

        if self.flag == 0:
            e_d = math.sqrt((self.x - self.x_goal)**2 + (self.y - self.y_goal)**2) #e_d = math.sqrt((self.x_goal)**2 + (self.y_goal)**2)
            if e_d <= 0.8:
                self.flag = 4
            else:
                # Align the camera with the goal orientation
                self.allign_camera()

        elif self.flag == 1:
            #  QUA POTREI CALCOLARE L'ERRORE RISPETTO A X_d Y_d INVECE CHE X E Y
            e_dd = math.sqrt((self.x - self.x_d)**2 + (self.y - self.y_d)**2) #
            if e_dd <= 0.2:
                # If the robot is close to the goal, transition to the next state
                self.flag = 2
                print(GOAL_PRINT_5)
            else:
                cmd_vel.linear.x = MAX_VEL
                cmd_vel.angular.z = 0.0
                self.publisher_.publish(cmd_vel)   

        elif self.flag == 2:
            # CONTROLLO ANGOLO ROBOT RISPETTO A MARKER
            # SE MOLTO DIVERSO, ALLINEO E VADO DRITTO
            # ALTRIMENTI MI ALLINEO MENTRE VADO
            theta = abs(self.theta)
            theta_marker = abs(self.theta_marker)
            if theta > math.pi:
                theta = theta - math.pi
            elif theta_marker > math.pi:
                theta_marker = theta_marker - math.pi
            #self.wait_for_input()
            if abs(theta - theta_marker) < 0.7:
                #print(3)
                self.flag = 3
            else:
                #print(4)
                self.flag = 4

        elif self.flag == 3:
            # Move towards the goal position
            e_d = math.sqrt((self.x - self.x_goal)**2 + (self.y - self.y_goal)**2)
            e_a = math.atan2(self.y_goal - self.y, self.x_goal - self.x)# - self.theta
            if e_a < 0:
                e_a = math.pi + (math.pi + e_a)
            #lin_control = 0.5 * e_d
            #ang_control = 0.1 * e_a
            lin_control = -self.linear_pid(e_d)
            ang_control = -self.angular_pid(0.05*e_a)
            cmd_vel.linear.x = lin_control
            if cmd_vel.linear.x > MAX_VEL:
                cmd_vel.linear.x = MAX_VEL
            cmd_vel.angular.z = ang_control
            if cmd_vel.angular.z > MAX_ANG_VEL:
                cmd_vel.angular.z = MAX_ANG_VEL
            self.publisher_.publish(cmd_vel)          
            if e_d <= 0.5:
                # If the robot is close to the goal, transition to the next state
                self.flag = 6
                self.stop()
                print(GOAL_PRINT_5)

        elif self.flag == 4:
            self.theta_d = math.atan2(self.y_goal - self.y, self.x_goal - self.x)
            if self.theta_d < 0:
                self.theta_d = math.pi + (math.pi + self.theta_d)
            self.allign_camera()

        elif self.flag == 5:
            e_d = math.sqrt((self.x - self.x_goal)**2 + (self.y - self.y_goal)**2)
            if e_d <= 0.5:
                # If the robot is close to the goal, transition to the next state
                self.flag = 6
                self.stop()
                print(GOAL_PRINT_5)
            else:
                cmd_vel.linear.x = 0.5 * e_d
                cmd_vel.angular.z = 0.0
                self.publisher_.publish(cmd_vel) 

        elif self.flag == 6:
            # Go backwards to allow the next turn
            self.flag = 7
            cmd_vel.linear.x = - MAX_VEL
            cmd_vel.angular.z = 0.0
            self.publisher_.publish(cmd_vel)  
            time.sleep(0.5)
            self.stop()
            return
        
        time.sleep(0.1)
        # Recursive call to continue the robot movement logic
        self.i += 1
        if self.i >= 350:
            self.flag = 8  
            time.sleep(0.5)
            self.stop()
            return
        
        self.robot_movement()

    def allign_camera(self):
        # Align the camera with the goal orientation
        # Next line is to handle the case where the angle wraps around from the maximum value (6.28) to the minimum value (0.0)
        diff = (self.theta_d - self.theta + 6.28) % 6.28
        if diff > 3.14:  # Half of the maximum range
            self.rotate(-1)
        else:
            self.rotate(1)
        if abs(self.theta_d - self.theta) < 0.05:
            print(GOAL_PRINT_4)
            self.stop()
            self.flag += 1

    def stop(self):
        # Stop the robot
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.publisher_.publish(cmd_vel)

    def rotate(self, sign):
        # Rotate the robot in place
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = sign * 0.7
        self.publisher_.publish(cmd_vel)

    def odom_callback(self, msg):
        # Callback for processing odometry data
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        # Convert quaternion to theta
        self.theta = math.atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)
        if self.theta < 0:
            self.theta = math.pi + (math.pi + self.theta)

    def calculate_desired_target(self):
        self.x_d = self.x_goal + (0.7* math.cos(self.theta_marker))
        self.y_d = self.y_goal + (0.7* math.sin(self.theta_marker))
        self.theta_d = math.atan2(self.y_d - self.y, self.x_d - self.x) #self.theta_d = math.atan2(self.y_d, self.x_d)
        if self.theta_d < 0:
            self.theta_d = math.pi + (math.pi + self.theta_d)

    # Create a rotation matrix around the Z-axis using NumPy
    def rotation_matrix_z(self, theta):
        c = np.cos(theta)
        s = np.sin(theta)
        rotation_matrix = np.array([[c, -s, 0],
                                    [s, c, 0],
                                    [0, 0, 1]])
        return rotation_matrix

    # Create a transformation matrix using translation and rotation
    def transformation_matrix(self, x, y, theta):
        translation_vector = np.array([x, y, 1])
        rotation_matrix = self.rotation_matrix_z(theta)  # You can concatenate other rotation matrices as needed

        # Create a 4x4 transformation matrix
        transformation_matrix = np.eye(3)
        transformation_matrix[:3, :3] = rotation_matrix
        transformation_matrix[:3, 2] = translation_vector

        return transformation_matrix
    # i fill it with self.x, self.y, self.theta
    # i obtain the relative position of the goal wrt the robot
    # i need the absolute position of the goal

    #def wait_for_input(self):
    #    while True:
    #        user_input = input("Quando vuoi andare avanti: ")
    #        if user_input:
    #            break

def main(args=None):
    rclpy.init(args=args)

    try:
        controller_logic = ControllerLogic()

        # Create a multi-threaded executor
        executor = MultiThreadedExecutor()

        # Add the node to the executor
        executor.add_node(controller_logic)

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

