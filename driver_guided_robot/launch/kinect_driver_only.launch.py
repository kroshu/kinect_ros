# Copyright 2020 Zoltán Rési
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.launch_description_sources.python_launch_description_source import PythonLaunchDescriptionSource  # noqa: E501
import launch_ros.actions
from launch_ros.actions import Node
import yaml


def load_file(package_name, file_path):

    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        print('Couldnt load file ' + absolute_file_path)
        return None


def load_yaml(package_name, file_path):

    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        print('Couldnt load yaml ' + absolute_file_path)
        return None


def generate_launch_description():

    # moveit_cpp.yaml is passed by filename for now since it's node specific
    moveit_iiwa_yaml_file_name = get_package_share_directory(
        'kinect_ros_trace_endpoint') + '/config/moveit_iiwa14.yaml'

    # Component yaml files are grouped in separate namespaces
    robot_description_config = load_file('urdflbriiwa14', 'urdf/urdflbriiwa14.urdf')
    robot_description = {'robot_description': robot_description_config}

    robot_description_semantic_config = load_file('urdflbriiwa14', 'urdf/urdflbriiwa14.srdf')
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}

    kinematics_yaml = load_yaml('kinect_ros_trace_endpoint', 'config/kinematics.yaml')

    ompl_planning_pipeline_config = {
        'planning_pipelines': ['ompl'],
        'ompl': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",  # noqa: E501
            'start_state_max_bounds_error': 0.1,
        },
    }
    ompl_planning_yaml = load_yaml(
        'kinect_ros_trace_endpoint', 'config/ompl_planning.yaml'
    )
    ompl_planning_pipeline_config['ompl'].update(ompl_planning_yaml)

    kinect_ros_trace_endpoint_node = Node(package='kinect_ros_trace_endpoint',
                                          executable='moveit_with_markerpos',
                                          name='moveit_with_markerpos',
                                          output='screen',
                                          arguments=['--ros-args', '--log-level', 'error'],
                                          remappings=[('fake_controller_joint_states',
                                                       'reference_joint_state')],
                                          parameters=[moveit_iiwa_yaml_file_name,
                                                      robot_description,
                                                      robot_description_semantic,
                                                      kinematics_yaml,
                                                      ompl_planning_pipeline_config
                                                      ])

    kuka_sunrise_dir = get_package_share_directory('kuka_sunrise')

    kuka_sunrise_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([kuka_sunrise_dir, '/launch/kuka_sunrise.launch.py'])
        )

    joint_controller = launch_ros.actions.LifecycleNode(
        package='robot_control', executable='joint_controller', output='screen',
        arguments=['--ros-args', '--log-level', 'info'],
        name='joint_controller', remappings=[('measured_joint_state', 'lbr_joint_state'),
                                             ('joint_command', 'lbr_joint_command')]
        )

    system_manager = launch_ros.actions.LifecycleNode(
        package='driver_guided_robot', executable='system_manager', output='screen',
        name='system_manager'
        )

    joint_state_publisher = Node(package='joint_state_publisher',
                                 executable='joint_state_publisher',
                                 name='joint_state_publisher',
                                 arguments=[get_package_share_directory(
                                     'urdflbriiwa14') + '/urdf/urdflbriiwa14.urdf'],
                                 output='log',
                                 parameters=[{'source_list': ['/lbr_joint_state'], 'rate': 100}])

    return LaunchDescription([
        kuka_sunrise_interface,
        system_manager,
        joint_controller,
        kinect_ros_trace_endpoint_node,
        joint_state_publisher
        ])
