import os

from ament_index_python.packages import get_package_share_directory

 

from launch import LaunchDescription

from launch.substitutions import LaunchConfiguration, Command

from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

 

def generate_launch_description():
    package_name='turtlebot'
    use_sim_time = LaunchConfiguration('use_sim_time')

 
    # Process the URDF file

    pkg_path = os.path.join(get_package_share_directory(package_name))

    xacro_file = os.path.join(pkg_path,'description/turtlebot_with_arm.xacro')

    robot_description_config = xacro.process_file(xacro_file)

 

    # Create a robot_state_publisher node

    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}

    node_robot_state_publisher = Node(

        package='robot_state_publisher',

        executable='robot_state_publisher',

        output='screen',

        parameters=[params]

    )

 

    gazebo = IncludeLaunchDescription(

        PythonLaunchDescriptionSource([os.path.join(

        get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py')]),

    )



    node_joint_broad = Node(package="controller_manager", executable="spawner",
                                        
                                        arguments=['joint_broad'])
    
    node_diff_drive_controller = Node(package="controller_manager", executable="spawner",
                                      arguments=["diff_cont"])
    

    node_trajectory_controller = Node(package="controller_manager", executable="spawner",
                                      arguments=["joint_trajectory_controller"])

 

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',

                        arguments=['-topic','robot_description',

                                   '-entity','turtlebot_with_arm'],

                        output='screen')
    

   

    return LaunchDescription([

        DeclareLaunchArgument(

        'use_sim_time',

        default_value = 'false',

        description = 'Use sim time if true'

        ),

        node_robot_state_publisher,

        gazebo,

        spawn_entity,

        node_diff_drive_controller,

        node_joint_broad,
        
        node_trajectory_controller

    ])