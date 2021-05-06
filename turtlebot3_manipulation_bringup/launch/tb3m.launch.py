#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
# Authors: Simon Honigmann

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
    omx_package_name = 'open_manipulator_x_robot'

    rviz_config = os.path.join(get_package_share_directory(
        omx_package_name), "launch", package_name + ".rviz")  # TODO

    # TODO: rework to use a single URDF source
    robot_description = os.path.join(get_package_share_directory(omx_package_name),
                                     "urdf", omx_package_name + ".urdf.xacro")

    ocr_usb_port = LaunchConfiguration('ocr_usb_port', default='/dev/ttyACM0')
    # omx_usb_port = LaunchConfiguration('omx_usb_port', default='/dev/ttyUSB1')

    controller_config = os.path.join(
        get_package_share_directory(
            omx_package_name), "controllers", "controllers.yaml"
    )

    tb3_param_dir = LaunchConfiguration(
        'tb3_param_dir',
        default=os.path.join(
            get_package_share_directory('turtlebot3_bringup'),
            'param',
            TURTLEBOT3_MODEL + '.yaml'))

    lidar_pkg_dir = LaunchConfiguration(
        'lidar_pkg_dir',
        default=os.path.join(get_package_share_directory('hls_lfcd_lds_driver'), 'launch'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=use_sim_time,
        description='Use simulation (Gazebo) clock if true')

    ocr_usb_port_arg = DeclareLaunchArgument(
        'ocr_usb_port',
        default_value=ocr_usb_port,
        description='Connected USB port with OpenCR')

    # omx_usb_port_arg = DeclareLaunchArgument(
    #     'omx_usb_port',
    #     default_value=omx_usb_port,
    #     description='Connected USB port with OpenManipulatorX')

    tb3_param_arg = DeclareLaunchArgument(
        'tb3_param_dir',
        default_value=tb3_param_dir,
        description='Full path to turtlebot3 parameter file to load')

    tb3_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': launch_configs['use_sim_time'][2]},
            {'robot_description': Command(['xacro ',
                                           robot_description,
                                           ' gazebo:=False'])}
        ],
        remappings=remappings,
        arguments=['--log-level', launch_configs['log_level'][2]]
    )

    hlds_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([lidar_pkg_dir, '/hlds_laser.launch.py']),
        launch_arguments=[('port', '/dev/ttyUSB0'), ('frame_id', 'base_scan')],
    )

    tb3_node = Node(
        package='turtlebot3_node',
        executable='turtlebot3_ros',
        parameters=[tb3_param_dir],
        arguments=['-i', open_cr_usb_port],
        output='screen')

    load_controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": Command(['xacro ', robot_description,
                                           ' gazebo:=False ' + xacro_args])}, controller_config],
        output={
            "stdout": "screen",
            "stderr": "screen",
        }
    )

    load_state_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_start_controller",
             "joint_state_controller"],
        output="screen",
        shell=True,
    )

    load_velocity_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_configure_controller",
             "velocity_controller"],
        output="screen",
        shell=True,
    )

    load_trajectory_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_start_controller",
             "joint_trajectory_controller"],
        output="screen",
        shell=True,
    )

    load_effort_controller = ExecuteProcess(
        cmd=["ros2", "control", "load_start_controller",
             "effort_controllers"],
        output="screen",
        shell=True,
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output={
            "stdout": "screen",
            "stderr": "log",
        }
    )

    return LaunchDescription([
        sim_time_arg,
        ocr_usb_port_arg,
        # omx_usb_port_arg,
        tb3_param_arg,
        tb3_state_publisher,  # TODO on this
        tb3_node,
        hlds_launch,
        tb3_node,
        load_controller_manager,
        load_state_controller,
        rviz_node,
        RegisterEventHandler(event_handler=OnProcessExit(
            target_action=load_state_controller,
            on_exit=[load_trajectory_controller, load_velocity_controller, load_effort_controller]
        )),
    ])
