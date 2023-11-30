konsole -e ros2 launch robot_urdf gazebo_aruco.launch.py &
sleep 5
konsole -e ros2 run assignment_pkg motor_control &
sleep 1
konsole -e ros2 run assignment_pkg robot_controller &
sleep 1
konsole -e ros2 run assignment_pkg robot_revolute_node 
sleep 1



