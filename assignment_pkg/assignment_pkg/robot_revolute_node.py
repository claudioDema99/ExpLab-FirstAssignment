import rclpy
import numpy as np

from rclpy.node import Node
from simple_pid import PID
from std_msgs.msg import Float64, Bool, Float64MultiArray
from rclpy.executors import MultiThreadedExecutor

'''
Node name: robot_revolute_node

Author: Claudio De Maria, Enrico Piacenti, Giancluca Galvagni

Publishers:

Subsceribers:


'''


class RobotController(Node):

    def __init__(self):
        super().__init__("robot_revolute_node")


        self.dt = 0.1

        self.sign = 1


        # Define current pose variables, at the beginning the camera is aligned with the marker
        self.current_angle = Float64()
        self.current_angle.data = 0.0

        # True if the camera is rotating for searching new markers, False if it is rotating to compensate the robot rotation
        self.modality = Bool()
        self.modality.data = True 

        # Define current velocity variables of the Robot, it is used to compensate the rotation of the robot
        self.angular_velocity = Float64()
        self.angular_velocity.data = 0.0

        
        # Define current angle of the camera to the rotation controller, at the beginning the camera is aligned with the marker
        self.theta_goal = Float64()
        self.theta_goal = 0.0

        # Timer to check the modality of the camera, created at the beginning to avoid errors
        self.modality_timer = self.create_timer(self.dt, self.camera_modality)

        # Define PID controllers for linear velocity and rotation
        self.ang_pid = PID(0.5, 0.01, 0.1)

        # Controllers update frequency (equivalent to publisher rate)
        self.ang_pid.sample_time = self.dt

        # Position and angular errors threshold and control saturation value
        self.ang_threshold = 0.01

        # Define counters, when 
        self.cnt_shutdown = 0

        # Define publishers
        self.publisher_rotation_goal = self.create_publisher(Float64, 'camera_theta_goal', 10)
        self.cmd_vel_pub = self.create_publisher(Float64MultiArray, "/joint_cam_controller/commands", 10)

        # Define subscribers
        self.subscriber_camera_modality = self.create_subscription(Bool, 'camera_on_off', self.camera_modality_callback, 10)
        self.subscriber_rotation = self.create_subscription(Float64, 'inverse_rotation', self.rotation_callback, 10)
        self.subscription_marker_reached = self.create_subscription (Bool,'marker_reached',self.marker_reached_callback, 10)


        self.get_logger().info("Revolute controller module initialized!")
        self.get_logger().info("Searching nearby markers...")



    def marker_reached_callback(self, msg: Bool):
        if msg.data == True:
            self.cnt_shutdown += 1
            self.get_logger().info("Marker reached!")
            if self.cnt_shutdown == 4:
                self.get_logger().info("All markers reached!")
                self.get_logger().info("Shutting down...")
                self.destroy_node()
                rclpy.shutdown()


    def rotation_callback(self, msg: Float64):
        self.angular_velocity.data = msg.data

    def camera_modality_callback(self, msg: Bool):
        self.modality.data = msg.data


    # Control loop cycle callback
    def camera_modality(self):
        """
        This function is called every dt seconds and checks if the camera is rotating or not.
        """
        print()
        print ("Ricerca Marker : ", self.modality.data)
        print()

        if self.modality.data == True:  # Camera is rotating

            # Increment the angle of the camera by 0.01 rad every dt seconds
            if 0.01 <= self.current_angle.data <= 6.27:  
                self.current_angle.data += 0.01 * self.sign
            elif self.current_angle.data < 0.01: # Avoid negative angles, enters when current angle is 0
                self.current_angle.data = 0.01
            elif self.current_angle.data > 6.27: # Avoid angles greater than 2pi, enters when current angle is 2pi
                self.current_angle.data = 6.27


            self.current_angle.data = round(self.current_angle.data, 3)

            # Change direction of rotation, it happends when current angle is 0 or 2pi 
            if self.current_angle.data == 0.0 or self.current_angle.data == 6.28: 
                self.sign = self.sign * (-1)

            # Publish command to rotate the joint
            cmd_msg = Float64MultiArray()
            cmd_msg.data = [self.current_angle.data]
            self.cmd_vel_pub.publish(cmd_msg)

            print("\nCamera is rotating: \nCurrent Angle is: {0}".format(self.current_angle.data))

        elif not self.modality.data :  # Rotation compensation is needed

            if self.angular_velocity.data < 0.0 and self.current_angle.data <= 6.23:
                self.current_angle.data += 0.05
                print(" Adding PLUS 0.05 to compensate rotation")
            elif self.angular_velocity.data > 0.0 and 0.05 <= self.current_angle.data:
                self.current_angle.data =  self.current_angle.data - 0.05
                print(" Substracting MINUS 0.05 to compensate rotation")


            # Publish command to rotate the joint
            cmd_msg = Float64MultiArray()
            cmd_msg.data = [self.current_angle.data]
            self.cmd_vel_pub.publish(cmd_msg)

            # Publish the current angle to the rotation controller
            theta_goal_msg = Float64()
            theta_goal_msg.data = self.current_angle.data
            self.publisher_rotation_goal.publish(theta_goal_msg)

            self.get_logger().info("\nCompensating angle from robot rotation, \nCurrent Angle is: {0}".format(self.current_angle.data))


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

        except Exception as e:
            print(f"Error in main loop: {str(e)}")

        finally:
            # Clean up
            executor.shutdown()
            controller_node.destroy_node()

    except Exception as e:
        print(f"Error in main: {str(e)}")

    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()



'''
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
            executor.shutdown()
            rclpy.shutdown()

    except Exception as e:
        print(f"Error in main: {str(e)}")

if __name__ == '__main__':
    main()
''' 