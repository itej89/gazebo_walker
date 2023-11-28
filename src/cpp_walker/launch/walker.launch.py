import os
import launch
from launch import launch_description_sources
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import launch_ros
from launch import actions
from launch_ros.actions import Node


from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    walker_dir = get_package_share_directory('cpp_walker')

    walker_gazebo_launch = actions.IncludeLaunchDescription(
        launch_description_sources.PythonLaunchDescriptionSource(
                walker_dir + '/launch/gazebo_launch.py'))
    

    walker_algo = Node(
        package="cpp_walker",
        executable="cpp_walker"
    )

    #Make launch description
    ld = launch.LaunchDescription()

    #Add nodes
    ld.add_action(walker_gazebo_launch)
    ld.add_action(walker_algo)

    return ld