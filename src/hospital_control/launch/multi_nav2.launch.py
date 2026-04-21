# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    # Set default model and paths
    turtlebot3_model = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    pkg_dir = get_package_share_directory('hospital_control')
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    # Configuration variables
    namespace = LaunchConfiguration('namespace', default='robot_2')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Map and Param paths
    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(pkg_dir, 'map', 'new_hospital.yaml'))

    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(pkg_dir, 'param', 'robot_2_burger.yaml'))

    # RViz config path
    rviz_config_dir = os.path.join(pkg_dir, 'rviz', 'tb3_navigation2.rviz')

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('namespace', default_value='robot_2'),
        DeclareLaunchArgument('map', default_value=map_dir),
        DeclareLaunchArgument('params_file', default_value=param_dir),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # Navigation Group with Namespace
        GroupAction([
            PushRosNamespace(namespace), 

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
                launch_arguments={
                    'map': map_dir,
                    'use_sim_time': use_sim_time,
                    'params_file': param_dir,
                    'use_namespace': 'false',
                    'autostart': 'true'
                }.items(),
            ),
        ]),

        # RViz2 Node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])