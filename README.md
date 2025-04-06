# rtab_sim: Indoor Mapping Drone Simulation Package

## Pre-requisites

1. Ubuntu 20.04 with ROS Noetic installation
2. Gazebo 11 Installation
3. ArduPilot SITL Installation
4. ArduPilot Gazebo Plugins Installation

## Package Build

To build the package: 

1.  Create a workspace folder in your ```$HOME``` directory:
    ```bash
    mkdir -p <your_ws>/src
    ```
    Replace ```your_ws``` with the name of your workspace.

2.  Clone the rtab_sim ROS Package:
    ```bash
    cd <your_ws>/src
    git clone https://github.com/fabeha-raheel/rtab_sim.git
    ```

3.  Build the ROS Package:
    ```bash
    cd ~/<your_ws>
    catkin_make
    ```

## ArduPilot Indoor Simulation Worlds
The package contains the following worlds that can be readily launched:
1.  AWS Small House World (with Iris Drone)
    ```bash
    roslaunch rtab_sim small_house_with_iris.launch
    ```
2.  Office Env World (with Iris Drone)
    ```bash
    roslaunch rtab_sim office_env_world_with_iris.launch
    ```
3.  Clearpath Robotics Office World (with Iris Drone)
    ```bash
    roslaunch rtab_sim office_cpr_world_with_iris.launch
    ```
4.  AWS Bookstore World (with Iris Drone)
    ```bash
    roslaunch rtab_sim bookstore_with_iris.launch
    ```

## Launching ArduPilot Indoor SITL Simulation
To launch ArduPilot SITL in indoor simulation environment, simply:
```bash
roslaunch rtab_sim indoor_ardupilot.launch
```

By default, the above launch file will utilize the AWS Small House World in Gazebo. 

To use a different world, pass the path of the desired world file as an argument as follows:
```bash
roslaunch rtab_sim indoor_ardupilot.launch world:=<path/to/desired/world/file>
```

## Launching ArduPilot Hector SLAM Simulation
The Hector SLAM package installation is required in order to launch the ArduPilot Hector SLAM simulation. If the Hector SLAM packages are not installed, follow the instructions given below.

### Hector SLAM Package Installation

1.  Install the Hector SLAM package in your ROS Workspace:
    ```bash
    cd ~/<your_ws>/src
    git clone https://github.com/tu-darmstadt-ros-pkg/hector_slam.git
    ```
2.  Modify the following files in the package using the content provided below:
    
    -   Modify ```$HOME/<your_ws>/src/hector_slam/hector_mapping/launch/mapping_default.launch``` file as shown below:
        ```xml
        <?xml version="1.0"?>

        <launch>
        <arg name="tf_map_scanmatch_transform_frame_name" default="scanmatcher_frame"/>
        <arg name="base_frame" default="base_link"/>
        <arg name="odom_frame" default="base_link"/>
        <arg name="pub_map_odom_transform" default="true"/>
        <arg name="scan_subscriber_queue_size" default="5"/>
        <arg name="scan_topic" default="scan"/>
        <arg name="map_size" default="2048"/>
        
        <node pkg="hector_mapping" type="hector_mapping" name="hector_mapping" output="screen">
            
            <!-- Frame names -->
            <param name="map_frame" value="map" />
            <param name="base_frame" value="$(arg base_frame)" />
            <param name="odom_frame" value="$(arg odom_frame)" />
            
            <!-- Tf use -->
            <param name="use_tf_scan_transformation" value="true"/>
            <param name="use_tf_pose_start_estimate" value="false"/>
            <param name="pub_map_odom_transform" value="$(arg pub_map_odom_transform)"/>
            
            <!-- Map size / start point -->
            <param name="map_resolution" value="0.050"/>
            <param name="map_size" value="$(arg map_size)"/>
            <param name="map_start_x" value="0.5"/>
            <param name="map_start_y" value="0.5" />
            <param name="map_multi_res_levels" value="2" />
            
            <!-- Map update parameters -->
            <param name="update_factor_free" value="0.4"/>
            <param name="update_factor_occupied" value="0.9" />    
            <param name="map_update_distance_thresh" value="0.4"/>
            <param name="map_update_angle_thresh" value="0.06" />
            <param name="laser_z_min_value" value = "-1.0" />
            <param name="laser_z_max_value" value = "1.0" />
            
            <!-- Advertising config --> 
            <param name="advertise_map_service" value="true"/>
            
            <param name="scan_subscriber_queue_size" value="$(arg scan_subscriber_queue_size)"/>
            <param name="scan_topic" value="$(arg scan_topic)"/>
            
            <!-- Debug parameters -->
            <!--
            <param name="output_timing" value="false"/>
            <param name="pub_drawings" value="true"/>
            <param name="pub_debug_output" value="true"/>
            -->
            <param name="tf_map_scanmatch_transform_frame_name" value="$(arg tf_map_scanmatch_transform_frame_name)" />
        </node>
            
        <node pkg="tf" type="static_transform_publisher" name="base_to_laser_broadcaster" args="0 0 0 0 0 0 base_link laser 100" />
        </launch>
        ```
    
    -   Modify ```$HOME/<your_ws>/src/hector_slam/hector_imu_attitude_to_tf/launch/example.launch``` file as shown below:
        ```xml
        <launch>
            <node pkg="hector_imu_attitude_to_tf" type="imu_attitude_to_tf_node" name="imu_attitude_to_tf_node" output="screen">
                <remap from="imu_topic" to="/mavros/imu/data" />
                <param name="base_stabilized_frame" type="string" value="base_stabilized" />
                <param name="base_frame" type="string" value="base_link" />
            </node>
        </launch>
        ```

    -   Modify ```$HOME/<your_ws>/src/hector_slam/hector_slam_launch/launch/tutorial.launch``` file as shown below:
        ```xml
        <?xml version="1.0"?>

        <launch>

            <arg name="geotiff_map_file_path" default="$(find hector_geotiff)/maps"/>

            <param name="/use_sim_time" value="true"/>

            <!-- <node pkg="rviz" type="rviz" name="rviz"
                args="-d $(find hector_slam_launch)/rviz_cfg/mapping_demo.rviz"/> -->

            <include file="$(find hector_mapping)/launch/mapping_default.launch"/>

            <include file="$(find hector_geotiff_launch)/launch/geotiff_mapper.launch">
                <arg name="trajectory_source_frame_name" value="scanmatcher_frame"/>
                <arg name="map_file_path" value="$(arg geotiff_map_file_path)"/>
            </include>

            <include file="$(find hector_imu_attitude_to_tf)/launch/example.launch"/>

        </launch>
        ```

3.  Build the Workspace
    ```bash
    cd ~/<your_ws>
    catkin_make
    source devel/setup.bash
    ```

### ArduPilot Hector SLAM Simulation
To launch the simulation:
```bash
roslaunch rtab_sim hector_slam_indoor.launch
```
You may pass the world argument with the above command.

## Controlling the Drone with Keyboard Teleoperation
The package also contains a module to control the ArduPilot drone using keyboard teleoperation. To control the drone, use the following commands in a separate terminal:
```bash
roslaunch rtab_sim teleop.launch
```

## Launching ArduPilot RTAB Map Simulation

### RTAB Map Package Installation

```bash
sudo apt install ros-$ROS_DISTRO-rtabmap-ros
```


### Testing RTAB Map with Depth AI Oak-D Depth Camera

Install the Depth AI Oak-D Depth camera ROS package:
```bash
sudo apt install ros-noetic-depthai-ros
```
For more information: https://docs.luxonis.com/software/ros/depthai-ros/

To resolve Rviz IMU plugin error:
```bash
sudo apt-get install ros-noetic-rviz-imu-plugin
```
To test RGBD-SLAM using RTAB Map with Oak-D depth camera:
1. Start the RTAB Map Package:
    ```bash
    roslaunch rtabmap_launch rtabmap.launch \
    args:="--delete_db_on_start" \
    rgb_topic:=/stereo_inertial_publisher/color/image \
    depth_topic:=/stereo_inertial_publisher/stereo/depth \
    camera_info_topic:=/stereo_inertial_publisher/color/camera_info \
    imu_topic:=/stereo_inertial_publisher/imu/data \
    frame_id:=oak-d_frame \
    approx_sync:=true \
    wait_imu_to_init:=true
    ```
2. Launch the Oak-D camera Node:
   ```bash
    roslaunch depthai_examples stereo_inertial_node.launch
    ```
3. Initialize the Odometry node:
   ```bash
    rosrun imu_filter_madgwick imu_filter_node \
   imu/data_raw:=/stereo_inertial_publisher/imu \
   imu/data:=/stereo_inertial_publisher/imu/data  \
   _use_mag:=false \
   _publish_tf:=false
    ```
For more information: https://wiki.ros.org/rtabmap_ros/Tutorials/HandHeldMapping


   
