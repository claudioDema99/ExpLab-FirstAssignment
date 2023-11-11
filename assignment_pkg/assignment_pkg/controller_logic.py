#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from ros2_aruco_interfaces.msg import ArucoMarkers
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class ControllerLogic(Node):

    def __init__(self):
        super().__init__('controller_logic')
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
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # INTERNAL VARIABLES
        self.distance = 0

    def timer_callback(self):
        msg = Twist()
        if self.distance > 0.2:
            msg.linear.x = 0.1
        elif self.distance <= 0.2:
            msg.linear.x = 0.0
        self.publisher_.publish(msg)

    def aruco_callback(self, msg):
        #self.get_logger().info('I heard mid: "%s"' % msg.marker_ids[0])
        self.get_logger().info('POSITION: "%f", "%f", "%f"' % (msg.poses[0].position.x, msg.poses[0].position.y, msg.poses[0].position.z))
        self.get_logger().info('ORIENTATION: "%f", "%f", "%f", "%f"' % (msg.poses[0].orientation.x, msg.poses[0].orientation.y, msg.poses[0].orientation.z, msg.poses[0].orientation.w))
        self.distance = msg.poses[0].position.z

    def odom_callback(self, msg):
        msg.pose.pose.position
        #self.get_logger().info('POSITION of ROBOT: "%f", "%f", "%f"' % (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z))


def main(args=None):
    rclpy.init(args=args)

    controller_logic = ControllerLogic()

    rclpy.spin(controller_logic)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    controller_logic.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
