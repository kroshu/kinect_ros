# kinect_ros (Arm tracking with Azure Kinect and LBR iiwa)
## The Azure Kinect driver node

Travis CI | SonarCloud
------------| ---------------
[![Build Status](https://api.travis-ci.com/kroshu/kinect_ros.svg?branch=main)](https://app.travis-ci.com/github/kroshu/kinect_ros) | [![Quality Gate Status](https://sonarcloud.io/api/project_badges/measure?project=kroshu_kinect_ros&metric=alert_status)](https://sonarcloud.io/dashboard?id=kroshu_kinect_ros)

### Prerequsites:
- Visual Studio (for colcon build)
- OpenCV (I downloaded the exe for version 3.4.4 from https://github.com/opencv/opencv/releases/tag/3.4.4)
- ros2 galactic (foxy didn't work for me, probably because of image_tansport package, which is included in galactic) -> download latest release version from official ros website
- cv_bridge package (in vision_opencv repo)
- python 3.8 **with debug binaries** (can be ticked in installation UI in advanced options)
- Boost installation (only prebuilt windows binaries worked for me: https://sourceforge.net/projects/boost/files/boost-binaries/1.74.0/boost_1_74_0-msvc-14.1-64.exe/download)
- Azure Kinect SDK
- Azure Kinect Body Tracking SDK
- the node source code itself (foxy-devel branch)
    - dnn_model.onnx file name might have to be changed in Findk4abt.cmake because of different versions
    - angles.h header file is needed (can be found at GitHub)

### Running the node
After building (colcon build in VS terminal) the node can be run with the 'ros2 run azure_kinect_ros_driver node --ros-args -p body_tracking_enabled:=true' command in a sourced terminal. (Or as a small cheat I changed the default value of the parameter "body_tracking_enabled" to true in the k4a_ros_device_params.h header, so that the 'ros2 run azure_kinect_ros_driver node' command starts the bodytracking too.)

### Connecting the Windows and Linux machines
Since ROS2 no bridge is needed between more machines, but a the communication method must be set up, which I achieved with the following environment variables:
- RMW_IMPLEMENTATION := rmw_fastrtps_cpp
- ROS_DOMAIN_ID := 0 #this ID is also set on the linux machine in bashrc, by another ID, both that, and the enable firewall ports must be changed (see later)
- FASTRTPS_DEFAULT_PROFILES_FILE := <*location_to_xml*>\com_interface.xml

where com_interface xml is:
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>CustomUDPTransport</transport_id>
            <type>UDPv4</type>
            <interfaceWhiteList>
                <address><insert-your-ip1-here></address>
				<address><insert-your-ip2-here></address>
            </interfaceWhiteList>
        </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="CustomUDPTransportParticipant">
        <rtps>
            <userTransports>
                <transport_id>CustomUDPTransport</transport_id>
            </userTransports>
        </rtps>
    </participant>
</profiles>
```

A few ports must also be enabled in firewall settings to let the messages through. By the domain ID of 0, I enabled ports 7410-7425 in both directions. (According to the documentation the exact port number is 7400 + 250 * domainID + 10 + 2 * participantID)
To try out the connection the demo nodes talker and listener are recommended.
The camera driver node publishes messages on the *body_tracking_data* topic, each message a MarkerArray consisting of 31 markers, one for each joint position.

## The kuka_sunrise driver node

The driver connects the Sunrise Fast Robot Interface with ROS (see details in Zoli's thesis). To start the simulation an Office PC is needed, while the ROS nodes should be run on a Linux machine. The Sunrise Workbench application is needed to configure the simulation (install, synchronize and set T1 in OfficeMode), but is only for visualisation purposes during the simulation.

### Launching the driver nodes: 

The kinect_driver_only.launch.py file (in package kinect_ros -> driver_guided_robot) starts the 3 lifecycle nodes for the driver (robot_control, robot_manger, system_manager) and the joint_controller node. (Additionally, the moveit_with_markerpos node is also started, which is to be connected to a node publishing position and orientations, but is not publishing anything if no other node is started.)
On the Office PC, the HMI must be connected to the ROS2_Control application, then the motion should be started. 
Configuring and activating the system_manager (*ros2 lifecycle set /system_manager configure*)  will also activate the other lifecycle nodes, the client state 4 indicates in  the terminal, that the FRI waits for commmands. Jogging is possible by clicking the start motion button again (pasusing the motion), this way the connection to the driver nodes can be tested.
