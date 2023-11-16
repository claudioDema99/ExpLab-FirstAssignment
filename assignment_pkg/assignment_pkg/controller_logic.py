#!/usr/bin/env python3
import rclpy
import math
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import String
from ros2_aruco_interfaces.msg import ArucoMarkers
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class ControllerLogic(Node):

    def __init__(self):
        super().__init__('controller_logic')
        ###################################################################################################################
        # SUBSCRIBER TO ARUCO_MARKERS
        self.subscription = self.create_subscription(
            ArucoMarkers,
            'aruco_markers',
            self.aruco_callback,
            10)
        self.subscription  # prevent unused variable warning
        # SUBSCRIBER TO ODOM
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.subscription  # prevent unused variable warning
        # PUBLISHER TO CMD_VEL AND TIMER FOR PUBLISHING THE MSGs
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        ###################################################################################################################
        # INTERNAL VARIABLES
        self.distance = 0
        self.id_marker = 0
        self.x = 0
        self.y = 0
        self.theta = 0
        # FLAG VARIABLES FOR EACH MARKER: when a marker is reached, set its flag to 1
        self.m11 = 0
        self.m12 = 0
        self.m13 = 0
        self.m15 = 0
        self.flag = 0
        ###################################################################################################################
        self.Kl = 0.4
        self.Ka = 1.0
        ###################################################################################################################
        self.count = 0
        self.x_array = [-1, -2, -3.4, -2.7]
        self.y_array = [2, 0.1, 1.3, 0.4]
        self.x_goal = self.x_array[self.count]
        self.y_goal = self.y_array[self.count]


    def timer_callback(self):# theta della camera a cui mi devo allineare
        # self.x_goal e self.y_goal sono la posizione del marker che devo raggiungere
        e_a = math.atan2(self.y_goal - self.y, self.x_goal - self.x) 
        if e_a < 0:
            e_a = math.pi + (math.pi + e_a)
        self.get_logger().info('ROBOT THETA : "%f"' % self.theta)
        self.get_logger().info('E_A : "%f"' % e_a)
        if abs(e_a - self.theta) > 0.1:
            if e_a > self.theta:
                self.rotate_counter()
            else:
                self.rotate_clockwise()
        else:
            if (self.flag==0):
                self.stop()
                self.flag += 1
            e_d = math.sqrt((self.x - self.x_goal)**2 + (self.y - self.y_goal)**2)
            if e_d <= 0.5:
                self.stop()
                print('REACHED')
                self.change_marker()
            else:
                cmd_vel = Twist()
                cmd_vel.linear.x = 0.4 * e_d
                cmd_vel.angular.z = 0.0
                self.publisher_.publish(cmd_vel)

    def change_marker(self):
        self.count += 1 
        self.x_goal = self.x_array[self.count]
        self.y_goal = self.y_array[self.count]

    def aruco_callback(self, msg):
        #self.get_logger().info('I heard mid: "%s"' % msg.marker_ids[0])
        #self.get_logger().info('ID: "%d"' % (msg.marker_ids[0]))
        #self.get_logger().info('POSITION: "%f", "%f", "%f"' % (msg.poses[0].position.x, msg.poses[0].position.y, msg.poses[0].position.z))
        #self.get_logger().info('ORIENTATION: "%f", "%f", "%f", "%f"' % (msg.poses[0].orientation.x, msg.poses[0].orientation.y, msg.poses[0].orientation.z, msg.poses[0].orientation.w))
        self.id_marker = msg.marker_ids[0]
        self.distance = msg.poses[0].position.z

    def odom_callback(self, msg):
        #msg.pose.pose.position
        #self.get_logger().info('POSITION of ROBOT: "%f", "%f", "%f"' % (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z))
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = float(msg.pose.pose.orientation.z)
        qw = msg.pose.pose.orientation.w
        #yaw = math.atan2(2.0*(q[1]*q[2] + q[3]*q[0]), q[3]*q[3] - q[0]*q[0] - q[1]*q[1] + q[2]*q[2])
        #yaw = math.atan2(2.0*(qy*qz + qw*qx), qw*qw - qx*qx - qy*qy + qz*qz)
        #pitch = math.asin(-2.0*(qx*qz - qw*qy))
        roll = math.atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)
        if roll<0:
            roll = math.pi + (math.pi + roll)
        #self.get_logger().info('R-P-Y: "%f"' % (roll))
        #self.get_logger().info('PORCA MERDA "%f", "%f", "%f", "%f"' % (qx, qy, qz, qw))
        self.theta = roll
        #self.get_logger().info('X: "%f"' % (self.x))
        #self.get_logger().info('Y: "%f"' % (self.y))
        #self.get_logger().info('Theta: "%f"' % (self.theta))
        
    def stop(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.publisher_.publish(cmd_vel)
        
    def rotate_clockwise(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = -0.5
        self.publisher_.publish(cmd_vel)
    
    def rotate_counter(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.5
        self.publisher_.publish(cmd_vel)
        

def main(args=None):
    rclpy.init(args=args)

    try:
    # declare the node constructor
        controller_logic = ControllerLogic()

        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(controller_logic)

        try:
            # pause the program execution, waits for a request to kill the node (ctrl+c)
            executor.spin()
        finally:
            executor.shutdown()
            controller_logic.destroy_node()

    finally:
        # shutdown the ROS communication
        rclpy.shutdown()


if __name__ == '__main__':
    main()
