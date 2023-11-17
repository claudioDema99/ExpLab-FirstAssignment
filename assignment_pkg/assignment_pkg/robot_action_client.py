import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import math

import cv2
from cv2 import aruco
from ros2_aruco_interfaces.msg import ArucoMarkers

from custom_action_interfaces_1explab.action import Camera
from custom_action_interfaces_1explab.action import MarkerPosition

class CameraActionClient(Node):

    def __init__(self):
        super().__init__('camera_action_client')
        # ACTION CLIENT to CAMERA
        self._action_client = ActionClient(self, Camera, 'camera')
        # ACTION CLIENT to CONTROLLER
        self._action_client = ActionClient(self, MarkerPosition, 'controller')
         # SUBSCRIBER TO ARUCO_MARKERS
        self.subscription_aruco = self.create_subscription(
            ArucoMarkers,
            'aruco_markers',
            self.aruco_callback,
            10)
        self.subscription_aruco  # prevent unused variable warning
        # PUBBLISHER TO CAMERA for rotating itself
        self.publisher_camera = self.create_publisher(bool, 'camera_on_off', 10)
        # Initialize the camera or video feed
        self.cap = cv2.VideoCapture(0)  # Use 0 for default camera, update accordingly
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

    # REQUEST to CONTROLLER to move to the goal position
    # goal sender with (x_goal, y_goal)'s marker from the camera
    def send_goal_position_marker(self, x_goal, y_goal):
        goal_msg = MarkerPosition.Goal()
        goal_msg.x_goal = x_goal
        goal_msg.y_goal = y_goal

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    # REQUEST to CAMERA to rotate itself 
    def send_goal_camera(self, theta):
        goal_msg = Camera.Goal()
        goal_msg.theta = theta

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    # callback for UPDATE the ARUCO MARKER'S INFO
    def aruco_callback(self, msg):
        # take the marker's id and position
        self.id_marker = msg.marker_ids[0]
        # take the marker's orientation
        qx = msg.poses[0].orientation.x
        qy = msg.poses[0].orientation.y
        qz = float(msg.poses[0].orientation.z)
        qw = msg.poses[0].orientation.w
        roll = math.atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)
        if roll<0:
            roll = math.pi + (math.pi + roll)      
        self.theta = roll
        # take the marker's position
        self.x_goal = msg.poses[0].position.x
        self.y_goal = msg.poses[0].position.y
        
    # CONTROLLER and DOING stuf with the camera
    def aruco_controller_area(self):
        # Capture a frame from the camera
        ret, frame = self.cap.read()

        # Convert the frame to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Define the ArUco dictionary and parameters
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters_create()

        # Detect markers in the image
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # Check if marker 11 is detected
        target_marker_id = 11
        
        if target_marker_id in ids:
            # Get the index of the target marker in the detected markers list
            target_marker_index = ids.tolist().index(target_marker_id)

            # Get the corners of the target marker
            target_marker_corners = corners[target_marker_index][0]

            # Calculate the area of the bounding box around the marker
            marker_area = cv2.contourArea(target_marker_corners)

            # Define the minimum and maximum allowed area
            min_area = 400  # 20x20 pixels

            # Check if the marker area is inside the minimun area
            if min_area < marker_area:
                # Marker is within the specified area
                print("Target marker is within the specified area.")
            else:
                print("Target marker is outside the specified area.")

        # Display the frame with detected markers
        frame_with_markers = aruco.drawDetectedMarkers(frame, corners)
        cv2.imshow("Frame with Markers", frame_with_markers)
        cv2.waitKey(0)
    
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
        self.get_logger().info('Result: {0}'.format(result.reached))
        rclpy.shutdown()

    # feedback callback we aren't using it
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))


def main(args=None):
    rclpy.init(args=args)

    action_client = CameraActionClient()

    action_client.send_goal(10)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
