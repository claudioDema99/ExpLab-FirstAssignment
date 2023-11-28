import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool, Float64, Float32MultiArray

from custom_action_interfaces_1explab.action import MarkerPosition
from ros2_aruco_interfaces.msg import ArucoMarkers

import math
import numpy as np 


class RobotActionClient(Node):

    def __init__(self):
        super().__init__('robot_action_client')
        # ACTION CLIENT to MOVE the ROBOT
        self._action_client = ActionClient(self, MarkerPosition, 'marker_position')
         # SUBSCRIBER TO ARUCO_MARKERS
        self.subscription_aruco = self.create_subscription(
            ArucoMarkers,
            'aruco_markers',
            self.aruco_callback,
            10)
        self.subscription_aruco  # prevent unused variable warning
        # SUBSCRIBER TO CORNERS OF THE MARKER
        self.subscription_corners = self.create_subscription(
            Float32MultiArray,
            'aruco_corners',
            self.corners_callback,
            10)
        self.subscription_corners  # prevent unused variable warning
        # PUBBLISHER TO CAMERA for rotating itself
        self.publisher_camera_onoff = self.create_publisher(Bool, 'camera_on_off', 10)
        # PUBBLISHER TO CAMERA the theta_goal
        self.publisher_camera_theta = self.create_publisher(Float64, 'camera_theta_goal', 10)  
        # TIMER for doing the controller logic
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)       
        ## INTERNAL VARIABLES ##
        # goal marker's info
        self.theta = 0
        self.x_goal = 0
        self.y_goal = 0
        self.id_marker = 0
        # marker's position in the list of markers seen by the camera
        self.position_marker = 0
        # corners of the marker searched when the marker is found
        self.corners_marker = []
        # list of the markers to reach
        self.goal_markers = [11, 12, 13, 15]
        # number of the marker reached
        self.reached_marker = 0
        # flag for the logic
        self.flag = 0
        # flag to check if the marker to reach is found by the camera
        self.flag_marker = 0
        
##############################################################################
############################# DEBUG FUNCTION #################################
##############################################################################
    def wait_for_input(self):
            while True:
                user_input = input("Quando vuoi andare avanti: ")
                if user_input:
                    break
                
##############################################################################
############################# FUNCTIONS ######################################
##############################################################################
        
    ## TIMER for the CONTROLLER LOGIC ##
    def timer_callback(self):
        self.id_marker = self.goal_markers[self.reached_marker] # take the marker's id to reach
        if self.flag == 0:
            #self.get_logger().info("Camera is rotating, the robot is waiting")
            self.rotation_camera_activation(True)
            self.aruco_controller_area()
        elif self.flag == 1:
            #self.get_logger().info("Camera is following the marker, the robot is moving")
            self.aruco_follow_marker()           
        # if the markers are reached go in home position
        if self.reached_marker == 4:
            self.get_logger().info("All markers are reached")
            self.flag = 3
            self.send_goal_position_marker(0, 0, 0)
            self.rotation_camera_activation(False)
            self.position_marker_camera(0)
        # end the program
        if self.reached_marker == 5:
            self.get_logger().info("Work done!")
            rclpy.shutdown()        
                
    ## ACTION CLIENT to MOVE the ROBOT ##
    def send_goal_position_marker(self):
        goal_msg = MarkerPosition.Goal()
        goal_msg.x_goal = (self.x_goal + 0.1) # add 10 cm because it is the distance between the camera frame and the robot frame
        goal_msg.y_goal = self.y_goal
        theta = self.theta
        # we need to convert the theta to respect the camera's frame into the marker's frame
        if math.pi*1/8 < theta < math.pi*3/8:
            theta = theta + math.pi
        elif math.pi*3/8 < theta < math.pi*5/8:
            theta = theta + math.pi
            if theta > math.pi*2:
                theta = theta - math.pi*2
        elif math.pi*5/8 < theta < math.pi*7/8:
            theta = theta - math.pi
        elif theta < math.pi*1/8 or theta > math.pi*7/8:
            theta = theta - math.pi
            if theta < 0:
                theta = theta + math.pi*2
        goal_msg.theta_goal = theta
        self.get_logger().info("Sending -> x_goal: {0}, y_goal: {1}, theta: {2}".format(goal_msg.x_goal, goal_msg.y_goal, goal_msg.theta_goal))
        self._action_client.wait_for_server()
        ###### DEBUG ######
        self.wait_for_input()
        ###################
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    # goal callback for goal reached 'accepted'
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    # result callback for goal reached 'succeeded'
    def get_result_callback(self, future):
        result = future.result().result
        if result.reached == True: # if the goal is reached so I reacheved a TRUE value from the server
            self.get_logger().info('Marker number {0} reached'.format(self.id_marker))
            self.flag = 0
            self.flag_marker = 0
            self.reached_marker += 1

    # feedback callback for goal reached 'feedback' but we don't use it
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        #self.get_logger().info('Distance to the marker goal: {0}'.format(feedback.distance_to_goal))
    
    ## PUBLISHER to CAMERA for rotating itself in searching mode ##
    def rotation_camera_activation(self, on_off):
        msg = Bool()
        msg.data = on_off
        self.publisher_camera_onoff.publish(msg)
        
    ## PUBLISHER to CAMERA the theta_goal in following mode ##
    def position_marker_camera(self, active):
        msg = Float64()
        if active == False: # if the marker is outside of the camera's field of view
            theta = 0.1 # go near the home position
        else:
            theta = self.theta
        msg.data = theta
        self.publisher_camera_theta.publish(msg)      
        
    ## callback for UPDATE the MARKER'S INFO ##
    def aruco_callback(self, msg):
        self.id_marker = self.goal_markers[self.reached_marker] # take the marker's id to reach
        # take the markers's id
        self.ids_marker = msg.marker_ids
        self.get_logger().warn('Markers found {0}'.format(self.ids_marker))
        # check if the marker is the one we are looking for
        if self.id_marker in self.ids_marker:
            self.flag_marker = 1
            #take the numeber inside the list
            self.position_marker = self.ids_marker.index(self.id_marker)
            # take the marker's orientation
            qx = msg.poses[self.position_marker].orientation.x
            qy = msg.poses[self.position_marker].orientation.y
            qz = float(msg.poses[self.position_marker].orientation.z)
            qw = msg.poses[self.position_marker].orientation.w
            roll = math.atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)
            if roll<0:
                roll = math.pi + (math.pi + roll)  
            # take the marker's info   
            self.theta = roll
            self.x_goal = msg.poses[self.position_marker].position.z
            self.y_goal = msg.poses[self.position_marker].position.x
            self.get_logger().info('x_goal: {0}, y_goal: {1}, theta: {2}'.format(self.x_goal, self.y_goal, self.theta))           
        else:
            # if the marker is not in the list we wait and then we check again
            self.flag_marker = 0
    
    ## callback for UPDATE the MARKER'S CORNERS ##
    def corners_callback(self, msg):
        if self.flag_marker == 1:
            # take the marker's corners
            big_data_corners = msg.data
            # put empty the list
            self.corners_marker = []
            # take the corners and put them in a list
            for i in range(8):
                self.corners_marker.append(big_data_corners[i+8*self.position_marker]) 
        else:
            self.corners_marker = []
     
    # WAIT for the MARKER to be in the AREA and then MOVE the ROBOT to the MARKER with the camera doing the motion 
    def aruco_controller_area(self):
        # Check if the position of the marker is in the area
        if self.flag_marker == 1 and len(self.corners_marker) != 0:                    
            # Calculate the area of the bounding box around the marker
            marker_area = self.calculate_rectangle_area(self.corners_marker)
            self.get_logger().info('Marker area: {0}'.format(marker_area))

            # Define the minimum and maximum allowed area
            min_area = 2000  # 20x20 pixels
            treshold = -600 # value to have the marker in the center of the camera's field of view (more perpendicolar to the camera)
            
            # Check if the marker area is inside the minimum area 
            if marker_area < min_area + treshold:
                self.flag = 1
                self.rotation_camera_activation(False)
                self.position_marker_camera(True)
                self.send_goal_position_marker()
            else:
                self.get_logger().info("Target marker is outside.")
    
    def calculate_rectangle_area(self, coordinates):
        # Calculate the area of the bounding box around the marker
        x1 = coordinates[0]
        x2 = coordinates[2]
        x3 = coordinates[4]
        x4 = coordinates[6]
        y1 = coordinates[1]
        y2 = coordinates[3]
        y3 = coordinates[5]
        y4 = coordinates[7]
        area = 0.5 * abs((x1*y2 + x2*y3 + x3*y4 + x4*y1) - (y1*x2 + y2*x3 + y3*x4 + y4*x1))
        return area
    
    # FOLLOW the MARKER with the camera doing the motion
    def aruco_follow_marker(self):
       # Check if the position of the marker is in the area
        if self.flag_marker == 1 and len(self.corners_marker) != 0:                    
            # Calculate the area of the bounding box around the marker
            marker_area = self.calculate_rectangle_area(self.corners_marker)
            # Define the minimum and maximum allowed area
            min_area = 2000  # 20x20 pixels         
            # Check if the marker area is inside the minimum area 
            if marker_area < min_area:    
                # Marker is within the specified area
                self.position_marker_camera(True)
        else:
            # Marker is outside of the camera's field of view
            self.position_marker_camera(False)
            self.get_logger().info("Target marker is outside the cameras' range. Camera is going to the home position")
    

def main(args=None):
    rclpy.init(args=args)
    try:
        action_client = RobotActionClient()
        print("Robot Action Client node has been started")
        
        # Create the executor
        ececuter = MultiThreadedExecutor()
        
        # Add the node to the executor
        ececuter.add_node(action_client)
        print("Robot Action Client node has been added to the executor")
        
        try:
            while rclpy.ok():
                ececuter.spin_once()
        
        finally:
            # Destroy the node explicitly
            print('Destroying node...')
            ececuter.shutdown()
            action_client.destroy_node()
            rclpy.shutdown()
            
    except KeyboardInterrupt:
        # Destroy the node explicitly
        print('Interrupted by user')
        action_client.destroy_node()
        rclpy.shutdown()

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()