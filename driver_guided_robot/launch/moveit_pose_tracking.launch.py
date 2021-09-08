import os
import yaml
from launch import LaunchDescription
from launch.actions.execute_process import ExecuteProcess
import launch_ros.actions
from launch_ros.actions import Node
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch.launch_description_sources.python_launch_description_source import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import xacro


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    # Get URDF and SRDF
    robot_description_config = xacro.process_file(
        os.path.join(get_package_share_directory("moveit_servo_trace"),
                     "config",
                     "iiwa.urdf.xacro"))
    robot_description = {"robot_description": robot_description_config.toxml()}

    robot_description_semantic_config = load_file(
        'urdflbriiwa14', 'urdf/urdflbriiwa14.srdf'
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    # Get parameters for the Pose Tracking node
    pose_tracking_yaml = load_yaml("moveit_servo", "config/pose_tracking_settings.yaml")
    pose_tracking_params = {"moveit_servo": pose_tracking_yaml}

    # Get parameters for the Servo node
    servo_yaml = load_yaml(
        "moveit_servo_trace", "config/iiwa_simulated_config_pose_tracking.yaml" 
    )
    servo_params = {"moveit_servo": servo_yaml}

    kinematics_yaml = load_yaml(
        "moveit_servo_trace", "config/kinematics.yaml"
    )

    # RViz
    """rviz_config_file = get_package_share_directory('driver_guided_robot') + "/launch/kinect_ros_trace_endpoint.rviz"
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file, '--ros-args', '--log-level', 'info'],
        parameters=[robot_description, robot_description_semantic, kinematics_yaml]
    )"""

    # A node to publish world -> URDFLBRiiwa14RobotBase transform
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'URDFLBRiiwa14RobotBase']
    )
    
    # Publish TF
    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 name='robot_state_publisher',
                                 output='both',
                                 parameters=[robot_description])

    
    pose_tracking_node = Node(
        package="moveit_servo_trace",
        executable="moveit_servo_trace",
        output="screen",
        arguments=['--ros-args', '--log-level', 'warn'],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            pose_tracking_params,
            servo_params]
    )
    
    trajectory_transformer = Node(
        package='moveit_servo_trace', executable='trajectory_transformer', output='screen',
        name='trajectory_transformer'
        )
    
    # Joint state publisher
    joint_state_publisher = Node(package='joint_state_publisher',
                                 executable='joint_state_publisher',
                                 name='joint_state_publisher',
                                 arguments=[get_package_share_directory(
                                     'urdflbriiwa14') + '/urdf/urdflbriiwa14.urdf'],
                                 output='log',
                                 parameters=[{'source_list': ['/lbr_joint_state']}])
                                 
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

        
    return LaunchDescription(
        [
            kuka_sunrise_interface,
            system_manager,
            joint_controller,
            static_tf,
            robot_state_publisher,
            joint_state_publisher,
            pose_tracking_node,
            trajectory_transformer                    
        ]
    )
