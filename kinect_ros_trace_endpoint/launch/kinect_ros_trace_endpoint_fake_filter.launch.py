# Copyright 2020 Gergely Kovács
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

    ompl_planning_pipeline_config = {'ompl': {
        'planning_plugin': 'ompl_interface/OMPLPlanner',
        'request_adapters': """default_planner_request_adapters/AddTimeOptimalParameterization
        default_planner_request_adapters/FixWorkspaceBounds
        default_planner_request_adapters/FixStartStateBounds
        default_planner_request_adapters/FixStartStateCollision
        default_planner_request_adapters/FixStartStatePathConstraints""",
        'start_state_max_bounds_error': 0.1}}
    ompl_planning_yaml = load_yaml('kinect_ros_trace_endpoint', 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['ompl'].update(ompl_planning_yaml)

    trajectory_execution = {'moveit_manage_controllers': True,
                            'trajectory_execution.allowed_execution_duration_scaling': 1.2,
                            'trajectory_execution.allowed_goal_duration_margin': 0.5,
                            'trajectory_execution.allowed_start_tolerance': 0.01}

    controllers_yaml = load_yaml('kinect_ros_trace_endpoint', 'config/fake_controllers.yaml')

    fake_controller = {'moveit_fake_controller_manager': controllers_yaml,
                       'moveit_controller_manager':
                       'moveit_fake_controller_manager/MoveItFakeControllerManager'}

    kinect_ros_trace_endpoint_node = Node(package='kinect_ros_trace_endpoint',
                                          executable='moveit_with_markerpos',
                                          name='moveit_with_markerpos',
                                          output='screen',
                                          remappings=[('fake_controller_joint_states',
                                                       'reference_joint_state'),
                                                      ('markers', 'body_tracking_data')],
                                          parameters=[moveit_iiwa_yaml_file_name,
                                                      robot_description,
                                                      robot_description_semantic,
                                                      kinematics_yaml,
                                                      ompl_planning_pipeline_config,
                                                      trajectory_execution,
                                                      fake_controller])
    # RViz
    rviz_config_file = get_package_share_directory(
        'kinect_ros_trace_endpoint') + '/launch/kinect_ros_trace_endpoint.rviz'
    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config_file],
                     parameters=[robot_description,
                                 robot_description_semantic])

    # Static TF
    static_tf = Node(package='tf2_ros',
                     executable='static_transform_publisher',
                     name='static_transform_publisher',
                     output='log',
                     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world',
                                'URDFLBRiiwa14RobotBase'])

    # Publish TF
    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 name='robot_state_publisher',
                                 output='both',
                                 parameters=[robot_description])

    # Joint state publisher
    joint_state_publisher = Node(package='joint_state_publisher',
                                 executable='joint_state_publisher',
                                 name='joint_state_publisher',
                                 arguments=[get_package_share_directory(
                                     'urdflbriiwa14') + '/urdf/urdflbriiwa14.urdf'],
                                 output='log',
                                 parameters=[{'source_list': ['/reference_joint_state']}])

    return LaunchDescription([static_tf, robot_state_publisher, rviz_node,
                              joint_state_publisher, kinect_ros_trace_endpoint_node])
