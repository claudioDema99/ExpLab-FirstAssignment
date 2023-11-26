import rclpy
import numpy as np

from rclpy.node import Node
from simple_pid import PID
from std_msgs.msg import Float64, Bool, Float64MultiArray
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor

import time


class RobotController(Node):


    def __init__(self):
        super().__init__("robot_revolute_node")

        # Define current pose variables, at the beginning the camera is aligned with the marker
        self.current_angle = Float64()
        self.current_angle.data = 0.0

        self.target = Bool()
        self.target.data = False  # True if a new target has been received

        self.ready = Bool()
        self.ready.data = False   # True if the PID is ready to control the robot

        self.dt = 0.1

        self.modality = Bool()
        self.modality.data = False
        

        self.sign = 1

        self.theta_goal = 0.0

        self.modality_timer = self.create_timer(self.dt, self.camera_modality) # Timer to check the modality of the camera, created at the beginning to avoid errors


        self.theta_goal = 0.0  

        # Define PID controllers for linear velocity and rotation
        self.ang_pid = PID(30.0, 2, 7.5)
        
        # Controllers update frequency (equivalent to publisher rate)
        self.ang_pid.sample_time = self.dt

        # Position and angular errors threshold and control saturation value
        self.ang_threshold = 0.01
        
        # Camera subscriber, it tells you the angular misalignment from theta to the goal and it tells you how the node should
        # behave (if it should rotate or not) based on the bool of the camera_on_off

        self.subscriber_camera_modality = self.create_subscription(Bool, 'camera_on_off', self.camera_modality_callback, 10)

        self.subscriber_camera_theta_goal = self.create_subscription(Float64, 'camera_theta_goal', self.camera_theta_callback, 10)

        # Cmd velocity publisher
        self.cmd_vel_pub = self.create_publisher(Float64MultiArray, "/joint_cam_controller/commands", 10)

        self.get_logger().info("Revolute controller module initialized!")
        self.get_logger().info("Searching nearby markers...")





    # I receive the theta_goal from the camera and I store it in a variable
    def camera_theta_callback(self, msg: Float64):

        #self.get_logger().info("Target.data is: {0})".format(self.target.data))
        if self.ready.data == True:  # it is True if a goal target has been received
            self.get_logger().error('Received new target orientation (Theta: {0})'.format(msg.data))
            # Store new target orientation (goal)
            self.theta_goal = msg.data
            self.target.data = True  # I don't need to receive another target yet


    def camera_modality_callback(self, msg: Bool):
            self.modality.data = msg.data
            self.get_logger().warn("Camera modality from callback is: {0})".format(self.modality.data))
            
            


    # Control loop cycle callback
    def camera_modality(self):

        self.get_logger().info("Camera modality is: {0})".format(self.modality.data))
        if self.modality.data == True: #Camera is rotating 

            #time.sleep(0.1)
            # Incrementally increase the target angle
            self.current_angle.data += 0.01 * self.sign

            self.current_angle.data = round(self.current_angle.data, 3)

            if (self.current_angle.data == 6.28) or (self.current_angle.data == 0.0):
                self.sign = self.sign * (-1)
                self.get_logger().info("Inverting Rotation...")

            # Publish command to rotate the joint
            cmd_msg = Float64MultiArray()
            cmd_msg.data = [self.current_angle.data]
            self.cmd_vel_pub.publish(cmd_msg)
            self.get_logger().info('(Sign: {0})'.format(self.sign))
            self.get_logger().info(' (Current_angle of rotation: {0})'.format(self.current_angle.data))


            # Log the current angle
            self.get_logger().info("Camera is rotating...")

        elif self.modality.data == False:
            self.ready.data = True
            if self.target.data == True:
                self.ang_pid.setpoint = self.theta_goal
                # Reset PID internals to clear previous errors
                self.ang_pid.reset()
                self.get_logger().info("Target selected, keeping an eye on it..")
                self.target.data = False
                self.timer = self.create_timer(self.dt, self.control_loop_callback)

        else:
            self.get_logger().error("Something went wrong with the camera modality" )
            
            
        



                

            
    

    # Control loop cycle callback
    def control_loop_callback(self):
        
            # Compute remaining errors
        ang_error = abs(self.current_angle.data - self.ang_pid.setpoint)

        print()
        #self.get_logger().info("Current Angle: {0})".format(self.current_angle.data))
        self.get_logger().info("Theta goal: {0})".format(self.ang_pid.setpoint))

        #self.get_logger().info("Remaining error: {0})".format(ang_error))

        # Compute rotation control based on the target rotation

        if ang_error < 6.28:
            ang_control = self.ang_pid(self.current_angle.data) * self.dt

            # Update the current angle based on the PID control
            self.current_angle.data += ang_control

        else:
            self.get_logger().warn(" Angle error is too much {0}".format(ang_error))

        # Publish command velocity message
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [self.current_angle.data]
        self.cmd_vel_pub.publish(cmd_msg)

            # If pose reached, exit the loop
        if ang_error < self.ang_threshold:
            self.get_logger().info('Waypoint reached')
            

    

        




def main(args=None):
    rclpy.init(args=args)

    try:
        controller_node = RobotController()

        # Create a multi-threaded executor
        executor = MultiThreadedExecutor()

        # Add the node to the executor
        executor.add_node(controller_node)

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

