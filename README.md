# Experimental Robotics Laboratory - Assignment 1

C. Demaria (s5433737)  
G. Galvagni (s)  
E. Piacenti (s)

## Description

In this assignment, we are going to use **ROS2** in order to control a mobile robot in a 3D simulation environment called *Gazebo*.  
We have the robot starting in the position (0, 0), and four markers with IDs 11, 12, 13, and 15.  
The markers have the following meanings:  
Marker 11 -> rotate until you find marker 12; then reach marker 12  
Marker 12 -> rotate until you find marker 13; then reach marker 13  
Marker 13 -> rotate until you find marker 15; then reach marker 15  
Marker 15 -> done!  
The robot used in this simulation has been appropriately modified as illustrated.

## The Robot

We have created a versatile mobile platform equipped with essential components for navigation and perception. The main chassis, defined by the *link_chassis* element, forms the central structure of the robot, housing key components and providing stability. Attached to the chassis are two wheels, *link_right_wheel* and *link_left_wheel*, each connected through continuous joints (*joint_right_wheel* and *joint_left_wheel*), enabling smooth and continuous motion.  
<br/>
A distinctive feature of this robot is its camera system, comprised of *link_camera_rot* and *camera_link*. The camera is mounted on a rotating joint (*joint_camera_rot*), allowing for dynamic orientation adjustments. This capability enhances the robot's perception and interaction with its environment.
<br/>
<figure>
<img src="readme_image/robot.png" style="width:40%">
</figure>
<br/>

## Logic of the program 



## The Nodes
### ROBOT CONTROL

The ROS2 node RobotControl for controlling a robot's motion based on the detection of ArUco markers through a camera. The node follows a marker-based navigation approach, utilizing marker information to guide the robot's movements.

### Subscribers

1. **Aruco Markers Subscriber (aruco_markers):**
   - Listens for information about ArUco markers detected by the camera.
   - Updates the internal state with marker IDs.
2. **Aruco Corners Subscriber (aruco_corners):**
   - Listens for updates on the corners of detected ArUco markers.
   - Updates the internal state with the corners' coordinates.
     
### Publishers

1. **Camera On/Off Publisher (camera_on_off):**
   - Publishes a `Bool` message to activate or deactivate camera rotation for searching ArUco markers.
2. **Marker Reached Publisher (marker_reached):**
   - Publishes a `Bool` message indicating whether the robot has reached the targeted ArUco marker.

### Timer

- Utilizes a timer to execute the main controller logic periodically.

### Internal Variables

- `id_marker:` ID of the current ArUco marker to reach.
- `position_marker:` Position of the current marker in the list of markers seen by the camera.
- `corners_marker:` Corners of the currently targeted ArUco marker.
- `goal_markers:` List of ArUco markers to reach.
- `reached_marker:` Number of markers successfully reached.
- `flag:` Flag used for the main logic to switch between camera rotation and marker following.
- `flag_marker:` Flag to check if the targeted marker is found by the camera.

### Controller Logic

1. Camera Rotation Mode (flag == 0):
   - Activates camera rotation and waits for a targeted ArUco marker to be found.
   - When a marker is found, transitions to marker following mode.
2. Marker Following Mode (flag == 1):
   - Follows the targeted ArUco marker using camera feedback.
   - Checks if the marker is within a specified area to consider it reached.
   - Publishes messages to indicate marker reaching status.
3. Completion Condition:
   - Terminates the node when all specified ArUco markers are successfully reached.


## Custom message and action


## Install and run

First of all, you need to run the master by typing:

    roscore

To install the module, you need to go inside the `src` folder of your ROS workspace and run the following command:

    git clone https://github.com/claudioDema99/RT1-SecondAssignment

and from the root directory of your ROS workspace run the command:

    catkin_make

To run the program, you need to have installed in your system the program **xterm**. To install it, run:

    sudo apt-get install xterm

Finally, to run the code, type the following command:

    roslaunch assignment_2_2022 assignment1.launch

## Possible improvements

The first thing you notice when running the program is that when a desired position is entered, it is not clear where in the arena this point is: we can simply put a marker inside the arena simulation in order to make this clear.  
In this way the user can also notice if the entered position is involuntarily near or inside a wall.  
Another possible improvement is modifying the algorithm that control the movement of the robot, because it seems to be very basic. For example, if the robot finds a wall while moving to the desired position, it overcomes it always going around it clockwise, sometimes getting further from the goal.


