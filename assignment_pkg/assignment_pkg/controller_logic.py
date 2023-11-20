#!/usr/bin/env python3
import rclpy
import math
import time
from rclpy.node import Node
from rclpy.action import ActionServer
from action_msgs.msg import GoalStatus
from my_interface.action import MarkerPosition
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

MAX_VEL = 0.5

class ControllerLogic(Node):

    def __init__(self):
        super().__init__('controller_logic')
        self.action_server = ActionServer(self, MarkerPosition, 'marker_position', self.execute_callback)
        # SUBSCRIBER TO ODOM
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.subscription  # prevent unused variable warning
        # PUBLISHER TO CMD_VEL
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # INTERNAL STATES VARIABLES
        self.x = 0
        self.y = 0
        self.theta = 0
        # GOAL VARIABLES
        self.x_goal = 0.0
        self.y_goal = 0.0
        self.theta_goal = 0.0
        # FLAG VARIABLE
        self.flag = 0

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        self.x_goal = goal.x_goal
        self.y_goal = goal.y_goal
        self.get_logger().info('    GOAL: X = "%f" Y = "%f" Theta = "%f"' % (self.x_goal, self.y_goal, self.theta_goal))
        self.theta_goal = goal.theta_goal
        goal_handle.succeed()
        self.robot_movement()
        if self.flag == 2:
            self.flag = 0
            result = MarkerPosition.Result()
            result.reached = True
            GoalStatus.STATUS_SUCCEEDED
            return result

    def robot_movement(self):
        if self.flag == 0:
            self.allign_camera()
        elif self.flag == 1: 
            e_d = math.sqrt((self.x - self.x_goal)**2 + (self.y - self.y_goal)**2)
            e_a = math.atan2(self.y_goal - self.y, self.x_goal - self.x) - self.theta
            if e_a < 0:
                e_a = math.pi + (math.pi + e_a)
            lin_control = 0.5 * e_d
            ang_control = 0.25 * e_a
            cmd_vel = Twist()
            cmd_vel.linear.x = lin_control
            if cmd_vel.linear.x > MAX_VEL:
                cmd_vel.linear.x = MAX_VEL
            cmd_vel.angular.z = ang_control
            self.publisher_.publish(cmd_vel)          
            if e_d <= 0.5:
                self.flag = 2
                self.stop()
                print("2) TARGET REACHED")
                return
        time.sleep(0.1)
        self.robot_movement()

    def allign_camera(self):
        if self.theta_goal > self.theta:
            self.rotate(1)
        else:
            self.rotate(-1)
        if abs(self.theta_goal - self.theta) < 0.2:
            print("1) ROBOT ALLIGNED WITH CAMERA")
            self.stop()
            self.flag = 1
        
    def stop(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.publisher_.publish(cmd_vel)

    def rotate(self, sign):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = sign * 0.7
        self.publisher_.publish(cmd_vel)
    
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = float(msg.pose.pose.orientation.z)
        qw = msg.pose.pose.orientation.w
        roll = math.atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)
        if roll<0:
            roll = math.pi + (math.pi + roll)
        self.theta = roll

def main(args=None):
    rclpy.init(args=args)

    try:
        controller_logic = ControllerLogic()

        try:
            rclpy.spin(controller_logic)

        finally:
            rclpy.shutdown()

    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
