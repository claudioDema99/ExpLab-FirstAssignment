import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool, Float64


from custom_action_interfaces_1explab.action import MarkerPosition
from ros2_aruco_interfaces.msg import ArucoMarkers

import math


class RobotActionClient(Node):

    def __init__(self):
        super().__init__('robot_action_client')
        # ACTION CLIENT to MOVE the ROBOT
        self._action_client = ActionClient(self, MarkerPosition, 'robot_controller')
         # SUBSCRIBER TO ARUCO_MARKERS
        self.subscription_aruco = self.create_subscription(
            ArucoMarkers,
            'aruco_markers',
            self.aruco_callback,
            10)
        self.subscription_aruco  # prevent unused variable warning
        # PUBBLISHER TO CAMERA for rotating itself
        self.publisher_camera_onoff = self.create_publisher(Bool, 'camera_on_off', 10)
        # PUBBLISHER TO CAMERA the theta_goal
        self.publisher_camera_theta = self.create_publisher(Float64, 'camera_theta_goal', 10)  
        # timer for doing the controller logic
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # INTERNAL VARIABLES
        self.theta = 0
        self.x_goal = 0
        self.y_goal = 0
        self.id_marker = 0
        self.goal_markers = [11, 12, 13, 15]
        self.marker_number = 0
        self.flag = 0
        self.flag_marker = 1
        self.cv_image = None

    ## TIMER for CONTROLLER the LOGIC ##
    def timer_callback(self):
        if self.flag == 0:
            print("Camera is rotating,looking for the marker and the robot is waiting")
            self.rotation_camera_activation(True)
            # Process the image for ArUco markers and robot motion
            self.aruco_controller_area()
        elif self.flag == 1:
            print("Camera is following the marker and the robot is moving")
            self.aruco_follow_marker()
        # if the markers are reached go in home position
        if self.marker_number == 4:
            print("All the markers are reached, the robot is going in home position")
            self.flag = 3
            self.send_goal_position_marker(0, 0, 0)
            self.position_marker_camera(0)
        
    ## REQUEST to CONTROLLER to move to the goal position##
    def send_goal_position_marker(self, x_goal, y_goal, theta):
        goal_msg = MarkerPosition.Goal()
        goal_msg.x_goal = x_goal
        goal_msg.y_goal = y_goal
        goal_msg.theta_goal = theta

        self._action_client.wait_for_server()

        #self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    # callback for checking if the info goal is reached
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    # result callback for goal reached 'reached'
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Marker number {0} reached'.format(self.goal_markers[self.marker_number]))
        self.marker_number += 1
        self.flag = 0

    # feedback callback we aren't using it
    #def feedback_callback(self, feedback_msg):
    #    feedback = feedback_msg.feedback
    #    self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))
    
    ## PUBLISHER to CAMERA for rotating itself ##
    def rotation_camera_activation(self, on_off):
        msg = Bool()
        msg.data = on_off
        self.publisher_camera_onoff.publish(msg)
        
    ## PUBLISHER to CAMERA the theta_goal ##
    def position_marker_camera(self, theta):
        msg = Float64()
        msg.data = theta
        self.publisher_camera_theta.publish(msg)      
        
    ## callback for UPDATE the ARUCO MARKER'S INFO ##
    def aruco_callback(self, msg):
        # Check if marker to detect is in the list of detected markers
        target_marker_id = self.goal_markers[self.marker_number]
        # take the markers's id
        self.ids_marker = msg.marker_ids
        # check if the marker is the one we are looking for
        if target_marker_id in self.ids_marker:
            self.flag_marker = 0
            #take the numeber inside the list
            position_marker = self.ids_marker.tolist().index(target_marker_id)
            # take the marker's orientation
            qx = msg.poses[position_marker].orientation.x
            qy = msg.poses[position_marker].orientation.y
            qz = float(msg.poses[position_marker].orientation.z)
            qw = msg.poses[position_marker].orientation.w
            roll = math.atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)
            if roll<0:
                roll = math.pi + (math.pi + roll)      
            self.theta = roll
            # take the marker's position
            self.x_goal = msg.poses[position_marker].position.x
            self.y_goal = msg.poses[position_marker].position.y
        else:
            # if the marker is not in the list we wait and then we check again
            self.get_logger().info('Marker not found')
            self.flag_marker = 1
    
     
    # WAIT for the MARKER to be in the AREA and then MOVE the ROBOT to the MARKER with the camera doing the motion 
    def aruco_controller_area(self):
        # Check if the callback is looking for the marker we are looking for
        self.id_marker = self.goal_markers[self.marker_number]
        # Check if the position of the marker is in the area
        if self.flag_marker == 0:
            # Get the index of the target marker in the detected markers list
            target_marker_index = self.ids_marker.tolist().index(self.id_marker)
            # Get the corners of the target marker
            target_marker_corners = []
            target_marker_corners.append(self.ids_marker[target_marker_index][0])
            target_marker_corners.append(self.ids_marker[target_marker_index][1])
            target_marker_corners.append(self.ids_marker[target_marker_index][2])
            target_marker_corners.append(self.ids_marker[target_marker_index][3])
                    
            # Calculate the area of the bounding box around the marker
            marker_area = self.calculate_rectangle_area(target_marker_corners)

            # Define the minimum and maximum allowed area
            min_area = 400  # 20x20 pixels
            treshold = 100  # value to increase the area for the marker to be in the area
            
            # Check if the marker area is inside the minimum area 
            if min_area + treshold < marker_area:
                # Marker is within the specified area
                self.flag = 1
                self.rotation_camera_activation(False)
                self.position_marker_camera(self.theta)
                self.send_goal_position_marker(self.x_goal, self.y_goal, self.theta)
                print("Target marker is within the specified area.")
            else:
                print("Target marker is outside the specified area.")
    
    def calculate_rectangle_area(self, coordinates):
        x1, y1 = coordinates[0]
        x2, y2 = coordinates[1]
        x3, y3 = coordinates[2]
        x4, y4 = coordinates[3]

        area = 0.5 * abs((x1*y2 + x2*y3 + x3*y4 + x4*y1) - (y1*x2 + y2*x3 + y3*x4 + y4*x1))
        return area
    
    # FOLLOW the MARKER with the camera doing the motion
    def aruco_follow_marker(self):
        # Check if the callback is looking for the marker we are looking for
        self.id_marker = self.goal_markers[self.marker_number]
        # Check if the position of the marker is in the area
        if self.flag_marker == 0:
            # Get the index of the target marker in the detected markers list
            target_marker_index = self.ids_marker.tolist().index(self.id_marker)
            # Get the corners of the target marker
            target_marker_corners = []
            target_marker_corners.append(self.ids_marker[target_marker_index][0])
            target_marker_corners.append(self.ids_marker[target_marker_index][1])
            target_marker_corners.append(self.ids_marker[target_marker_index][2])
            target_marker_corners.append(self.ids_marker[target_marker_index][3])
                    
            # Calculate the area of the bounding box around the marker
            marker_area = self.calculate_rectangle_area(target_marker_corners)

            # Define the minimum and maximum allowed area
            min_area = 400  # 20x20 pixels
            
            # Check if the marker area is inside the minimum area 
            if min_area < marker_area:    
                # Marker is within the specified area
                self.position_marker_camera(self.theta)
                print("Target marker is within the specified area.")
            else:
                # Marker is outside the specified area
                self.position_marker_camera(0)
                print("Target marker is outside the specified area. Camera is going to the home position")
    

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
