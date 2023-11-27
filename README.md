# Experimental Robotics Laboratory - Assignment 1

C. Demaria (s5433737)
G. Galvagni (s)
E. Piacenti (s)

## Description

In this assignment, we are going to use **ROS2** in order to control a mobile robot in a 3D simulation environment called *Gazebo*.  We have the robot starting in the position (0, 0), and four markers with IDs 11, 12, 13, and 15. 
The markers have the following meanings:
Marker 11 -> rotate until you find marker 12; then reach marker 12
Marker 12 -> rotate until you find marker 13; then reach marker 13
Marker 13 -> rotate until you find marker 15; then reach marker 15
Marker 15 -> done!
The robot used in this simulation has been appropriately modified as illustrated.

## The Robot



## Logic of the program 



## The Nodes



## The custom message and action


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


