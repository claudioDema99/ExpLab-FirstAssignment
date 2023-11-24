import rclpy
import numpy as np

from rclpy.node import Node
from simple_pid import PID
from std_msgs.msg import Float64, Bool, Float64MultiArray
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor


class RobotController(Node):

    target = Bool()
    target.data = False

    def __init__(self):
        super().__init__("robot_revolute_node")

        # Define current pose variables, at the beginning the camera is aligned with the marker
        self.current_angle = Float64()
        self.current_angle.data = 0.0


        self.dt = 0.05

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

        if self.target.data == True:
            self.get_logger().info('Received new target orientation (Theta: {0})'.format(msg.data))
            # Store new target orientation (goal)
            self.theta_goal = msg.data
            self.target.data = False

    # Control loop cycle callback
    def camera_modality_callback(self, msg: Bool):
        if msg.data == True: ## if the camera is on, the camera should rotate
            # Incrementally increase the target angle
            self.current_angle.data += 0.01

            if self.current_angle.data == 6.28:
                self.current_angle.data = 0.0
            # Publish command to rotate the joint
            cmd_msg = Float64MultiArray()
            cmd_msg.data = [self.current_angle.data]
            self.cmd_vel_pub.publish(cmd_msg)

            # Log the current angle
            self.get_logger().info("Camera is rotating...")
        else:
            self.get_logger().info("Target selected, keeping an eye on it..")

            self.target.data = True

            # Set new PID setpoints for orientation
            self.ang_pid.setpoint = self.theta  

            # Reset PID internals to clear previous errors
            self.ang_pid.reset()

            # Start timer for control loop callback
            self.timer = self.create_timer(self.dt, self.control_loop_callback)

            
            '''
            #self.timer = self.create_timer(self.dt, self.control_loop_callback)
            while (msg.data == False):

                # Compute remaining errors
                self.ang_error = abs(msg.data - self.ang_pid.setpoint)

                # Compute rotation control based on target rotation
                ang_control = self.ang_pid(self.current_angle.data) * self.dt
                # Publish command velocity message
                cmd_vel = Twist()
                cmd_vel.angular.z = ang_control
                self.cmd_vel_pub.publish(cmd_vel)
            '''
                

            
    

    # Control loop cycle callback
    def control_loop_callback(self):
        rate = self.create_rate(1 / self.dt)

        while not self.destroyed and not self.get_clock().is_shutdown():
            # Compute remaining errors
            ang_error = abs(self.current_angle.data - self.ang_pid.setpoint)

            self.get_logger().info("Remaining error: (rotation: {0})".format(ang_error))

            # Compute rotation control based on the target rotation
            ang_control = self.ang_pid(self.current_angle.data) * self.dt

            # Update the current angle based on the PID control
            self.current_angle.data += ang_control

            # Publish command velocity message
            cmd_msg = Float64MultiArray()
            cmd_msg.data = [self.current_angle.data]
            self.cmd_vel_pub.publish(cmd_msg)

            # If pose reached, exit the loop
            if ang_error < self.ang_threshold:
                self.get_logger().info('Waypoint reached')
                break

            rate.sleep()


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

