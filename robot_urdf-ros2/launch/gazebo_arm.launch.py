"""
Spawn Robot Description
"""
import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    test_robot_description_share = FindPackageShare(package='robot_urdf').find('robot_urdf')
    default_model_path = os.path.join(test_robot_description_share, 'urdf/m2wr_arm2.xacro')
    

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )
    
    arm_01_controller = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_01_controller"],
    )
    
    arm_02_controller = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_02_controller"],
    )
    
    arm_03_controller = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_03_controller"],
    )
    
    arm_04_controller = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_04_controller"],
    )
    
    broad = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_broad"],
    )

    
    
   

    # GAZEBO_MODEL_PATH has to be correctly set for Gazebo to be able to find the model
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-entity', 'my_test_robot', '-topic', '/robot_description'],
                        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument(name='model', default_value=default_model_path,
                                    description='Absolute path to robot urdf file'),
        robot_state_publisher_node,
        joint_state_publisher_node,
        spawn_entity,
        arm_01_controller,
        arm_02_controller,
        arm_03_controller,
        arm_04_controller,
        broad,
        ExecuteProcess(
            cmd=['gazebo', '--verbose','worlds/empty.world', '-s', 'libgazebo_ros_factory.so'],
            output='screen')
    ])
