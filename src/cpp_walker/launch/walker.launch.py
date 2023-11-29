import os
import launch
from launch import launch_description_sources
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess

from launch.conditions import IfCondition

from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

import launch_ros
from launch import actions
from launch_ros.actions import Node


from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    walker_dir = get_package_share_directory('cpp_walker')

    # Gazebo launch node
    walker_gazebo_launch = actions.IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
                walker_dir + '/launch/gazebo_launch.py')
    )
    
    # Walker node
    walker_algo = Node(
        package="cpp_walker",
        executable="cpp_walker"
    )

    # Launch argument that enables or disables ros2 bag recording
    is_record_bag=  DeclareLaunchArgument(
        "is_record_bag", 
        default_value='false',
        description='Determines if ros bag record should be enabled.'
    )

    # Launch argument that sets the ros2 bag recording path
    bag_file_path = DeclareLaunchArgument(
        "bag_file_path", 
        default_value='rosbag/talker',
        description='Determines the location to save the bag file.'
    )

    # Launch ros2 bag recorder
    proc_ros_bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '-o', LaunchConfiguration('bag_file_path')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('is_record_bag'))
    )
    
    # Make launch description
    ld = launch.LaunchDescription()

    # Add nodes
    ld.add_action(walker_gazebo_launch)
    ld.add_action(walker_algo)
    ld.add_action(is_record_bag)
    ld.add_action(bag_file_path)
    ld.add_action(proc_ros_bag)

    return ld