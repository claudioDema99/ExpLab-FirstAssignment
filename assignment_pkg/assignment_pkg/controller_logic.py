#!/usr/bin/env python3
import rclpy
import math
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

    def timer_callback(self):
        a = 1
        b = 1
        #c = 0
        x_gol = 1.5
        y_gol = 1.5
        theta_gol = 1.9 # theta marker - pi greco..
        cmd_vel = Twist()
        if a*(x_gol-self.x)<0.1 or b*(y_gol-self.y)<0.1:
            self.stop_first()
            if theta_gol - self.theta > 0.05:# + c
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.5
            else:
                e_d = math.sqrt((self.x - 1.5)**2 + (self.y - 1.5)**2)
                cmd_vel.linear.x = float(e_d)
                cmd_vel.angular.z = 0.0
                if e_d <= 0.5:
                    cmd_vel.linear.x = 0.0
        else:
            e_a = theta_gol - self.theta
            e_d = math.sqrt((self.x - 1.5)**2 + (self.y - 1.5)**2)
            if e_d <= 0.1:
                e_d = 0.0
                e_a = 0.0
            if e_a <= 0.2:
                e_d = 0.5/0.4
                #c = 1.57
            # Compute control inputs
            lin_control = 0.4 * e_d
            ang_control = 0.4 * e_a
            # Build twist msg
            cmd_vel.linear.x = float(lin_control)
            cmd_vel.angular.z = float(ang_control)
        self.publisher_.publish(cmd_vel)
        self.get_logger().info('X: "%f"' % (self.x))
        self.get_logger().info('Y: "%f"' % (self.y))
        self.get_logger().info('Theta: "%f"' % (self.theta))
        """
        # theta gol = theta marker - pi greco
        theta_gol = 1.57
        x_gol = 1.5
        y_gol = 1.5
        if self.x > x_gol:
            if theta_gol - self.theta > 0.05:
                cmd_vel = Twist()
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.5
                self.publisher_.publish(cmd_vel)
                self.get_logger().info('X: "%f"' % (self.x))
                self.get_logger().info('Y: "%f"' % (self.y))
                self.get_logger().info('Theta: "%f"' % (self.theta))
            else:
                e_d = math.sqrt((self.x - 1.5)**2 + (self.y - 1.5)**2)
                cmd_vel = Twist()
                cmd_vel.linear.x = float(e_d)
                cmd_vel.angular.z = 0.0
                if e_d <= 0.5:
                    cmd_vel.linear.x = 0.0
                self.publisher_.publish(cmd_vel)
                self.get_logger().info('X: "%f"' % (self.x))
                self.get_logger().info('Y: "%f"' % (self.y))
                self.get_logger().info('Theta: "%f"' % (self.theta))
        else:
            e_a = theta_gol - self.theta
            # compute distance error
            e_d = math.sqrt((self.x - 1.5)**2 + (self.y - 1.5)**2)
            # compute angular error
            #e_a = math.atan2(1.5 - self.y, 1.5 - self.x) - self.theta 
            if e_d <= 0.1:
                e_d = 0.0
                e_a = 0.0
            # Compute control inputs
            lin_control = 0.3 * e_d
            ang_control = 0.3 * e_a
            # Build twist msg
            cmd_vel = Twist()
            cmd_vel.linear.x = float(lin_control)
            cmd_vel.angular.z = float(ang_control)
            # Publish it
            self.publisher_.publish(cmd_vel)
            self.get_logger().info('   E_D: "%f"' % (e_d))
            self.get_logger().info('E_A: "%f"' % (e_a))
            self.get_logger().info('X: "%f"' % (self.x))
            self.get_logger().info('Y: "%f"' % (self.y))
            self.get_logger().info('Theta: "%f"' % (self.theta))
        
        msg = Twist()
        if self.m11 == 0: # we are looking for the marker 11
            if self.distance > 0.2 and self.id_marker == 11:
                msg.linear.x = -0.1
            elif self.distance <= 0.2 and self.id_marker == 11:
                msg.linear.x = 0.0
                self.m11 = 1
        elif self.m11 == 1 and self.m12 == 0: # we are looking for the marker 12
            if self.distance > 0.2 and self.id_marker == 12:
                msg.linear.x = 0.1
            elif self.distance <= 0.2 and self.id_marker == 12:
                msg.linear.x = 0.0
                self.m12 = 1
        elif self.m12 == 1 and self.m13 == 0: # we are looking for the marker 13
            if self.distance > 0.2 and self.id_marker == 13:
                msg.linear.x = 0.1
            elif self.distance <= 0.2 and self.id_marker == 13:
                msg.linear.x = 0.0
                self.m13 = 1
        elif self.m13 == 1 and self.m15 == 0: # we are looking for the marker 15
            if self.distance > 0.2 and self.id_marker == 15:
                msg.linear.x = 0.1
            elif self.distance <= 0.2 and self.id_marker == 15:
                msg.linear.x = 0.0
                self.m15 = 1
        self.publisher_.publish(msg)"""

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
    
    def stop_first(self):
        if self.flag == 0:
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
            self.publisher_.publish(cmd_vel)
            self.flag += 1
        else:
            return

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
