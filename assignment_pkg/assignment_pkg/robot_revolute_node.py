#!/usr/bin/python3
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
        self.modality.data = True # True if the camera is rotating, False if it is not rotating
        
        #self.waypoint_reached = Bool()
        #self.waypoint_reached.data = True # True if the waypoint has been reached, False if it has not been reached yet

        self.begin_compensation = Bool()
        self.begin_compensation.data = False

        self.angular_velocity = Float64()
        self.angular_velocity.data = 0.0


        self.sign = 1

        self.theta_goal = Float64()
        self.theta_goal = 0.0

        self.modality_timer = self.create_timer(self.dt, self.camera_modality) # Timer to check the modality of the camera, created at the beginning to avoid errors


        # Define PID controllers for linear velocity and rotation
        self.ang_pid = PID(0.5, 0.01, 0.1)
        
        # Controllers update frequency (equivalent to publisher rate)
        self.ang_pid.sample_time = self.dt

        # Position and angular errors threshold and control saturation value
        self.ang_threshold = 0.01
        


        # Define publishers 
        self.publisher_roatation_goal = self.create_publisher(Float64, 'camera_theta_goal', 10)

        self.cmd_vel_pub = self.create_publisher(Float64MultiArray, "/joint_cam_controller/commands", 10)


        # Define subscribers
        self.subscriber_camera_modality = self.create_subscription(Bool, 'camera_on_off', self.camera_modality_callback, 10)

        self.subscriber_rotation = self.create_subscription(Float64, 'inverse_rotation', self.rotation_callback, 10)


        self.get_logger().info("Revolute controller module initialized!")
        self.get_logger().info("Searching nearby markers...")





    def rotation_callback(self, msg: Float64):
        self.angular_velocity.data = msg.data
        


    '''      # I receive the theta_goal from the camera and I store it in a variable
    def camera_theta_callback(self, msg: Float64):

        #self.get_logger().info("Target.data is: {0})".format(self.target.data))
        self.get_logger().error('Received new target orientation (Theta: {0})'.format(msg.data))
        # Store new target orientation (goal)
        self.theta_goal = msg.data
        self.target.data = True  # I don't need to receive another target yet
        self.ang_pid.reset()
        self.waypoint_reached.data = False    '''



    def camera_modality_callback(self, msg: Bool):
            self.modality.data = msg.data


            

            #self.get_logger().warn("Camera modality from callback is: {0})".format(self.modality.data))
            
            


    # Control loop cycle callback
    def camera_modality(self):  ## Questa funzione è chiamata ogni dt secondi, e controlla se la camera sta ruotando o no

        #self.get_logger().info("Searching Markers...") # solo questo ad inizo programma.


        if self.modality.data == True: # Camera is rotatiing

            self.begin_compensation.data = True

            # step is the increment of the angle of the camera of 0.01 rad every dt seconds
            if self.current_angle.data >= 6.27:
                self.current_angle.data = 6.26
                self.get_logger().error('Angle is too much')
            elif self.current_angle.data <= 0.01:
                self.current_angle.data = 0.02
                self.get_logger().error('Angle is too low')

            self.current_angle.data += 0.02 * self.sign

            self.current_angle.data = round(self.current_angle.data, 3)

            if (self.current_angle.data == 0.0) or (self.current_angle.data == 6.28):
                self.sign = self.sign * (-1)
                #self.get_logger().info("Inverting Rotation...")


            # Publish command to rotate the joint
            cmd_msg = Float64MultiArray()
            cmd_msg.data = [self.current_angle.data]
            self.cmd_vel_pub.publish(cmd_msg)



            self.get_logger().info("\nCamera is rotating, \nCurrent Angle is: {0}".format(self.current_angle.data))
            #self.get_logger().info(' ( Waypoint_reached: {0})'.format(self.waypoint_reached.data))
            #self.get_logger().info('(Sign: {0})'.format(self.sign))

            # Log the current angle
            

        elif self.modality.data == False : # Camera is not rotating and waypoint has not been reached yet

            '''              print('I am waiting for a new target, I am not rotating')
            self.ang_pid.sample_time = self.dt
            self.ang_pid.setpoint = self.theta_goal
            self.target.data = False
            self.timer = self.create_timer(self.dt, self.control_loop_callback)   '''

            #self.begin_compensation = True


            
            if self.angular_velocity.data < 0.0 and self.current_angle.data < 6.27:
                self.sign = +1    
                self.step = 0.025
                self.current_angle.data += self.step*self.sign
            elif self.angular_velocity.data > 0.0 and self.current_angle.data > 0.01:
                self.sign = -1
                self.step = 0.025
                self.current_angle.data += self.step*self.sign

            

            cmd_msg = Float64MultiArray()
            cmd_msg.data = [self.current_angle.data]
            self.cmd_vel_pub.publish(cmd_msg)

            theta_goal_msg = Float64()
            theta_goal_msg.data = self.current_angle.data
            self.publisher_roatation_goal.publish(theta_goal_msg)


            self.get_logger().info("\nCompensating angle from Dema, \nCurrent Angle is: {0}".format(self.current_angle.data))



            
            #else:
            #    print('I am waiting for another ...')
                #self.get_logger().error("Camera has stopped its rotation but target is not selected yet, waiting for a target..")
                # Per un problema probabilmente di diverse sequenze di esecuzione tra programma e timer, ogni tanto il programma stampa
                # questo messaggio anche se il target è stato selezionato, ma non è un problema perchè il programma funziona lo stesso

        else:
            self.get_logger().info("Something went wrong with the camera modality" )
            
            
            
    

    # Control loop cycle callback, it is called every dt seconds and it works if waypoint_reached is False
    def control_loop_callback(self):


        if self.waypoint_reached.data == True:            
            return
        
        # Compute remaining errors
        ang_error = abs(self.current_angle.data - self.ang_pid.setpoint)


        ## Debug
        log_message = (
            f"Current Angle: {self.current_angle.data}\n"
            f"Theta goal: {self.ang_pid.setpoint}\n"
            f"Remaining error: {ang_error}\n"
        )
        self.get_logger().info(log_message)


       
        # Compute rotation control based on the target rotation, changed 
        ang_control = np.clip(self.ang_pid(self.current_angle.data) * self.dt, -0.05, 0.05)
        #self.get_logger().warn(" Angle control is: {0}".format(ang_control))


        # Update the current angle based on the PID control
        if self.current_angle.data > 6.27:
            self.current_angle.data = 6.27
            self.get_logger().error('Angle is too much')
            
        elif self.current_angle.data <= 0.01:
            self.current_angle.data = 0.01
            self.get_logger().error('Angle is too low')
            
        self.current_angle.data += ang_control
        self.get_logger().warn(" Current angle is: {0}".format(self.current_angle.data))
        # Publish command velocity message
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [self.current_angle.data]
        self.cmd_vel_pub.publish(cmd_msg)

        # If pose reached, exit the loop
        if ang_error < self.ang_threshold:
            log_message += 'Waypoint reached\n'
            self.get_logger().info(log_message)            
            self.waypoint_reached.data = True
            return
            
            



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
