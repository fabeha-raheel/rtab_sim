## Chapter 1: Basic Concepts

## System Requirements

In order to be able to use the rtabmap_ros package to perform RGB-D SLAM, you need to have, at least, a Kinect-like sensor.  
  
Anyways, the recommended robot configuration is the following:

-   A 2D laser which provides  **sensor_msgs/LaserScan messages**.
-   Odometry (IMU, wheel encoders, ...) which provides a  **nav_msgs/Odometry**  message.
-   A calibrated Kinect-like sensor compatible with openni_launch, openni2_launch or freenect_launch ros packages.

Two of the most common setups in order to perform RGB-D Slam are the following:

#### Kinect + Odometry + 2D laser

![enter image description here](https://github.com/rwbot/RTAB-Map-Notes/blob/master/images/2/rtabmap_config.png?raw=true)


#### Kinect + Odometry + Fake 2D laser from Kinect

![](https://github.com/rwbot/RTAB-Map-Notes/blob/master/images/2/rtabmap_config_fakelaser.jpg?raw=true)



## Data Visualization (RViz)

Add the necessary elements to RViz in order to visualize the following data. 

a) RGB Image  
b) Depth Cloud  
c) Laser Scans

##### You will have to set the Fixed Frame to "base_link" in order to be able to visualize the data.  
![](https://github.com/rwbot/RTAB-Map-Notes/blob/master/images/2/sensors_rviz.png?raw=true)


## Launching RTAB-Map

The rtabmap_ros package, as many other ROS packages, has a set of parameters that you need to set up in order to properly launch it. So let's analyze some of the most important ones. For that, you can have a look at the launch file named  _demo_launch_file.launch_  that is contained in the  _turtlebot_rtab_  package. So, find the mentioned launch file, open it, and have a look at it. It should look like this:  
  
![](https://github.com/rwbot/RTAB-Map-Notes/blob/master/images/2/demo_launch_file.png?raw=true)

Before explaining some of these parameters, you need to know this one thing: when we talk about rtabmap_ros related parameters, we need to differentiate two kind of parameters: ROS Parameters and RTAB-Map Parameters.

## ROS Parameters

The ROS parameters are for connecting the RTAB-Map library with ROS.

-   **frame_id (string, default: "base_link")**: The frame attached to the mobile base.
-   **odom_frame_id (string, default: "")**: The frame attached to odometry. If empty, rtabmap will subscribe to odom topic to get odometry. If set, odometry is got from tf (in this case, a covariance of 1 is used).
-   **subscribe_depth (bool, default: "true")**: Subscribe to depth image.
-   **subscribe_scan (bool, default: "false")**: Subscribe to laser scan.
-   **wait_for_transform (bool, default: "true")**: Wait (maximum wait_for_transform_duration sec) for transform when a tf transform is not still available.
-   **wait_for_transform_duration (double, default: 0.1)**: Wait duration for wait_for_transform. To avoid some possible errors, it is recommended to set this value to "0.2".
-   **database_path (string, default: "~/.ros/rtabmap.db")**: Path of the RTAB-Map's database.

## RTAB-Map Parameters

The RTAB-Map's parameters are those related to the RTAB-Map library.

-   **RGBD/NeighborLinkRefining**: Correct odometry using the input laser topic using ICP.
-   **RGBD/ProximityBySpace**: Find local loop closures based on the robot position in the map. It is useful when the robot, for example, is coming back in the opposite direction. With camera facing back, global loop closures cannot be found. So using the position and previously added laser scans to the map, we find the transform using ICP. Be aware that on large-scale mapping, this method should be disabled because when the odometry is very erroneous, local ICP could give wrong results (false loop closures).
-   **RGBD/AngularUpdate**: The robot should move to update the map (if not 0).
-   **RGBD/LinearUpdate**: The robot should move to update the map (if not 0).
-   **RGBD/OptimizeFromGraphEnd**: Here we optimized from the latest node added to the map instead of the first. By optimizing from the last, the last pose keeps it's value and all the previous poses are corrected according to it (so /odom and /map will always match together). By optimizing from the first, all the successive nodes are corrected according to the first one (so there will be a transformation between /odom and /map to correct the last pose added). However, by optimizing from the first: when some nodes are transferred, the first referential of the local map may change, resulting in momentary changes in robot/map position (which are annoying in teleoperation).
-   **Optimizer/Slam2D**: Do 2D graph optimization (only optimize x, y and yaw values).
-   **Reg/Strategy**: We chose ICP to refine global loop closures found with ICP using the laser scans.
-   **Reg/Force3DoF**: Force 3DoF registration: roll, pitch and z won't be estimated.
-   **Vis/MinInliers**: Minimum visual words inliers (after RANSAC transformation between images of a loop closure) to accept the transformation.
-   **Vis/InlierDistance**: From the RANSAC transformation computed, the maximum distance of the visual word inliers.
-   **Rtabmap/TimeThr**: The maximum time allowed for map update. When this threshold is reached, some nodes are transferred in the Long-Term Memory to satisfy real-time constraints. The map will be published without these nodes (until retrieved from Long-Term Memory to Working Memory).
-   **Mem/RehearsalSimilarity**: In the papers, it is referred as the Weight Update threshold. If consecutive nodes have similar images (over this threshold), they are merged together, increasing the total weight of the merged node. The weighting mechanism is used for the memory management approach (less weighted nodes will be transferred first to Long-Term Memory).

### Complete look at all the RTAB-Map parameters here: [rtabmap_ros ](http://wiki.ros.org/rtabmap_ros)

---

## Subscribed Topics

In order to work properly, the  _rtabmap_ros_  package needs to subscribe to some topics to get the data it needs. This topics are the following:

-   **rgb/image (sensor_msgs/Image)**: RGB/Mono image. Should be rectified when subscribe_depth is true.
-   **rgb/camera_info (sensor_msgs/CameraInfo)**: RGB camera metadata.
-   **depth/image (sensor_msgs/Image)**: Registered depth image. Required if parameter subscribe_depth is true.
-   **scan (sensor_msgs/LaserScan)**: Laser scan stream. Required if parameter subscribe_scan is true.
-   **odom (nav_msgs/Odometry)**: Odometry stream. Required if parameters subscribe_depth or subscribe_stereo are true and odom_frame_id is not set.

Note that in each system, the name of these topics may vary. This makes (almost always) mandatory to do  **remappings**  from this topics to the ones that are actually providing this data in our system. You can see how this is done within the previously shown launch file.  
  
![](https://github.com/rwbot/RTAB-Map-Notes/blob/master/images/2/rtabmap_remap.png?raw=true)


## Arguments

You can also specify the following arguments to the rtabmap_ros package:

-   **"--delete_db_on_start"**: Delete the database before starting, otherwise the previous mapping session is loaded.
-   **"--udebug"**: Show RTAB-Map's debug/info/warning/error logs.
-   **"--uinfo"**: Show RTAB-Map's info/warning/error logs.
-   **"--params"**: Show RTAB-Map's parameters related to this node and exit.
-   **"--params-all"**: Show all RTAB-Map's parameters and exit.


**`demo_launch_file.launch`**
```xml
<launch>
    <group ns="rtabmap">
        <node name="rtabmap" pkg="rtabmap_ros" type="rtabmap" output="screen" args="--delete_db_on_start">
            <param name="frame_id" type="string" value="base_link"/>
            
            <param name="subscribe_depth" type="bool" value="true"/>
            <param name="subscribe_scan" type="bool" value="true"/>
            
            <!--<remap from="odom" to="/base_controller/odom"/>-->
            <remap from="odom" to="/odom"/>
            <!--<remap from="scan" to="/base_scan"/>-->
            <remap from="scan" to="/kobuki/laser/scan"/>
            
            
            <!--<remap from="rgb/image" to="/camera/rgb/image_rect_color"/>-->
            <remap from="rgb/image" to="/camera/rgb/image_raw"/>
            <!--<remap from="depth/image" to="/camera/depth_registered/image_raw"/>-->
            <remap from="depth/image" to="/camera/depth/image_raw"/>
            <remap from="rgb/camera_info" to="/camera/rgb/camera_info"/>
            
            <!-- RTAB-Map's parameters -->
            <param name="RGBD/NeighborLinkRefining" type="string" value="true"/>
            <param name="RGBD/ProximityBySpace"     type="string" value="true"/>
            <param name="RGBD/AngularUpdate"        type="string" value="0.01"/>
            <param name="RGBD/LinearUpdate"         type="string" value="0.01"/>
            <param name="RGBD/OptimizeFromGraphEnd" type="string" value="false"/>
            <param name="Optimizer/Slam2D"          type="string" value="true"/>
            <param name="Reg/Strategy"              type="string" value="1"/> 
            <param name="Reg/Force3DoF"             type="string" value="true"/>
            <param name="Vis/MinInliers"            type="string" value="5"/>
            <param name="Vis/InlierDistance"        type="string" value="0.1"/>
            <param name="Rtabmap/TimeThr"           type="string" value="700"/>
            <param name="Mem/RehearsalSimilarity"   type="string" value="0.45"/>
        </node>
    </group>
</launch>
```
#

## Mapping Mode

So, now that you have already seen how to properly launch the rabmap_ros package, it is time to give it some use!! And the first thing we are going to do is to generate a Map of the environment. So... let's go! For that, you'll need to create a new launch file in order to launch both RTAB-Map and the Navigation system.  
  
For building this new launch file, you should take these 2 things into account:
1.  Regarding the Navigation System, you will just need to launch the  _move_base_  node, since the SLAM process will be handled by the  _rtabmap_ros_package itself.
3.  By default, the rtabmap_ros package publishes the grid map that it's created into a  _/grid_map_  topic. In the Navigation system, though, the grid map is read from the  _/map topic_. So, you'll need to do the propper remap here.

`rtab_mapping.launch`
```xml
<launch>
  
  <arg name="database_path"     default="rtabmap.db"/>
  <arg name="args"              default=""/>
  
  <arg name="wait_for_transform"  default="0.2"/> 
  
  <!-- Navigation stuff (move_base) -->
  <include file="$(find turtlebot_navigation)/launch/includes/move_base_rtab.launch.xml"/>
  
  <!-- Mapping -->
  <group ns="rtabmap">

    <node name="rtabmap" pkg="rtabmap_ros" type="rtabmap" output="screen" args="$(arg args)">
      <param name="database_path"       type="string" value="$(arg database_path)"/>
      <param name="frame_id"            type="string" value="base_footprint"/>
      <param name="odom_frame_id"       type="string" value="odom"/>
      <param name="wait_for_transform_duration"  type="double"   value="$(arg wait_for_transform)"/>
      <param name="subscribe_depth"     type="bool"   value="true"/>
      <param name="subscribe_scan"      type="bool"   value="true"/>
    
      <!-- inputs -->
      <remap from="scan"            to="/kobuki/laser/scan"/>
      <remap from="rgb/image"       to="/camera/rgb/image_raw"/>
      <remap from="depth/image"     to="/camera/depth/image_raw"/>
      <remap from="rgb/camera_info" to="/camera/rgb/camera_info"/>
      
      <!-- output -->
      <remap from="grid_map" to="/map"/>
    
      <!-- RTAB-Map's parameters: do "rosrun rtabmap rtabmap (double-dash)params" to see the list of available parameters. -->
      <param name="RGBD/ProximityBySpace"        type="string" value="true"/>   
      <param name="RGBD/OptimizeFromGraphEnd"    type="string" value="false"/>  
      <param name="Kp/MaxDepth"                  type="string" value="4.0"/>
      <param name="Reg/Strategy"                 type="string" value="1"/>      
      <param name="Icp/CoprrespondenceRatio"     type="string" value="0.3"/>
      <param name="Vis/MinInliers"               type="string" value="5"/>      
      <param name="Vis/InlierDistance"           type="string" value="0.1"/>    
      <param name="RGBD/AngularUpdate"           type="string" value="0.1"/>    
      <param name="RGBD/LinearUpdate"            type="string" value="0.1"/>    
      <param name="Rtabmap/TimeThr"              type="string" value="700"/>
      <param name="Mem/RehearsalSimilarity"      type="string" value="0.30"/>
      <param name="Optimizer/Slam2D"             type="string" value="true"/>
      <param name="Reg/Force3DoF"                type="string" value="true"/>   
      
    </node>
   
  </group>
</launch>
```

### To Map:
- Launch the rtabmap and move_base nodes:
`roslaunch rtabrw rtab_mapping.launch`
- Launch rviz:
`roslaunch rtabmap_ros demo_turtlebot_rviz.launch`
- Launch teleop to control:
`roslaunch turtlebot_teleop keyboard_teleop.launch`

---
After a mapping session as above, a database is saved here ~/.ros/rtabmap.db. Into this database, the rtabmap_ros package stores, for instance, images from the mapping session that will be later used for  **detecting loop closures**.

In order to better explain how this works, let's access to this database. The rtabmap_ros package offers a tool that allows us to visualize the content of this database: the  **RTAB-Map's Database Viewer**. You can open this tool using the following command:

`rtabmap-databaseViewer ~/.ros/rtabmap.db`

 ![enter image description here](https://github.com/rwbot/RTAB-Map-Notes/blob/master/images/3/rtab_database_viewer.png?raw=true)

This images you are seeing right now are the different images that have been taken during the Mapping session. If you move the "Id" scroll button that appears at the bottom of the screen, you will move through all the different images that are stored in the database.

But, as you may have noticed, most of the images have some strange yellow marks on them (for now, forget about the pink marks). What are these strange yellow marks? Can you guess? Well, basically,  **this overlapping yellow disks are marking/highlightning the key features of each image**. And... how are the key features of each image selected? That's another great question!  
  
This visual features used by RTAB-Map are using some popular techniques from computer vision including like SIFT, SURF, BRIEF, FAST, BRISK, ORB or FREAK. Most of these algorithms look for large changes in intensity in different directions around a point in the image. If you check into the different images, you will notice that there are no yellow discs centered on the homogeneous parts of the image such as the walls or the floor. Instead, the discs are inserted into areas where there are changes in intensity such as the corners. Corner-like features tend to be stable properties of a given location and can be easily detected even under different lighting conditions or when the robot’s view is from a different angle or distance from an object.

The rtabmap_ros package records these collections of visual features in memory as the robot maps the area. At the same time, a machine learning technique known as the “bag of words model” looks for patterns in the features that can then be used to classify the various images as belonging to one location. For instance, there may be a hundred different video frames like the one shown above but from slightly different viewpoints that all contain visual features similar enough to assign to the same location.

That's awesome, right? But now... what are those pink discs that appear in some cases? Could you find out what they mean? Can you detect in which cases they appear? I'll give you some minutes...

So what? Did you discovered anything? Well... let's solve the mystery! The  **pink discs indicate visual features that two images have in common**. For instance, if you select in the Database Viewer 2 images with the same Id (so that are the same image), there should be lots of visual features in common between both images, right?  

![](https://github.com/rwbot/RTAB-Map-Notes/blob/master/images/3/rtab_database_common.png?raw=true)  

On the other hand, if we select 2 images that don't have key features in common, we won't get any pink disk.  

![](https://github.com/rwbot/RTAB-Map-Notes/blob/master/images/3/rtab_database_different.png?raw=true)  
Based on the number of shared features and their geometric relations to one another, we can determine if the two views should be assigned to the same location or not. In this way, only a subset of the visual features needs to be stored in long term memory while still being able to recognize a location from many different viewpoints. As a result, RTAB-Map can map out large areas such as an entire building or an outdoor campus without requiring an excessive amount of memory storage or processing power to create or use the map.  
  
So... what do you say? Amazing, right? Well, now you know a little bit better how the whole process works, let's move to the Localization section!

---
---

## Localization Mode

So, after we have already mapped the environment, we can then relaunch the rtabmap_ros package within the localization mode.

In order to launch the package in localization mode, you need to take into account the following:

-   The RTAB-Map Parameter  **_Mem/IncrementalMemory_**  has to be set to false, and  **_Mem/InitWMWithAllNodes_**  has to be set to true.

The recommended way of doing this is adding to the previous launch file the required parameters for the localization mode, and then adding a condition to them.
`<param     if="$(arg name_of_the_argument)" name="NameOfTheParam" type="string" value="false"/>`

This way, you can launch the localization mode by just adding a parameter to the launch file you've already created. Like this:
`roslaunch rtabmap_ros demo_turtlebot_mapping.launch localization:=true`

When the localization mode is launched, you can move the robot around the environment until it can relocalize in the previous map. Then, **the 2D map would re-appear again when a loop closure is found**.
* In order to be able to relocalie itself, the package must detect a loop closure. So, you should move the robot to a place where you know your database has recognizable images from.  
* When your robot localizes itself and the whole 2D map appears, if the 3D map doesn't appear, you can just click on the "Download map" option in Rtabmap cloud panel.

![](https://github.com/rwbot/RTAB-Map-Notes/blob/master/images/3/download_map.png?raw=true)


- Robot without localizing himself:

![](https://github.com/rwbot/RTAB-Map-Notes/blob/master/images/3/un_localized.png?raw=true)


- Robot localized:

![](https://github.com/rwbot/RTAB-Map-Notes/blob/master/images/3/localized.png?raw=true)

---
**`rtab_localization.launch`**

```xml
<launch>
  
  <arg name="database_path"     default="rtabmap.db"/>
  <arg name="localization"      default="false"/>
  <arg name="args"              default=""/>
  
  <arg name="wait_for_transform"  default="0.2"/> 
  
  <!-- Navigation stuff (move_base) -->
  <include file="$(find turtlebot_navigation)/launch/includes/move_base_rtab.launch.xml"/>
  
  <!-- Mapping -->
  <group ns="rtabmap">

    <node name="rtabmap" pkg="rtabmap_ros" type="rtabmap" output="screen" args="$(arg args)">
      <param name="database_path"       type="string" value="$(arg database_path)"/>
      <param name="frame_id"            type="string" value="base_footprint"/>
      <param name="odom_frame_id"       type="string" value="odom"/>
      <param name="wait_for_transform_duration"  type="double"   value="$(arg wait_for_transform)"/>
      <param name="subscribe_depth"     type="bool"   value="true"/>
      <param name="subscribe_scan"      type="bool"   value="true"/>
    
      <!-- inputs -->
      <remap from="scan"            to="/kobuki/laser/scan"/>
      <remap from="rgb/image"       to="/camera/rgb/image_raw"/>
      <remap from="depth/image"     to="/camera/depth/image_raw"/>
      <remap from="rgb/camera_info" to="/camera/rgb/camera_info"/>
      
      <!-- output -->
      <remap from="grid_map" to="/map"/>
    
      <!-- RTAB-Map's parameters: do "rosrun rtabmap rtabmap (double-dash)params" to see the list of available parameters. -->
      <param name="RGBD/ProximityBySpace"        type="string" value="true"/>   
      <param name="RGBD/OptimizeFromGraphEnd"    type="string" value="false"/>  
      <param name="Kp/MaxDepth"                  type="string" value="4.0"/>
      <param name="Reg/Strategy"                 type="string" value="1"/>      
      <param name="Icp/CoprrespondenceRatio"     type="string" value="0.3"/>
      <param name="Vis/MinInliers"               type="string" value="5"/>      
      <param name="Vis/InlierDistance"           type="string" value="0.1"/>    
      <param name="RGBD/AngularUpdate"           type="string" value="0.1"/>    
      <param name="RGBD/LinearUpdate"            type="string" value="0.1"/>    
      <param name="Rtabmap/TimeThr"              type="string" value="700"/>
      <param name="Mem/RehearsalSimilarity"      type="string" value="0.30"/>
      <param name="Optimizer/Slam2D"             type="string" value="true"/>
      <param name="Reg/Force3DoF"                type="string" value="true"/>   
      
      <!-- localization mode -->
      <param     if="$(arg localization)" name="Mem/IncrementalMemory" type="string" value="false"/>
      <param unless="$(arg localization)" name="Mem/IncrementalMemory" type="string" value="true"/>
      <param name="Mem/InitWMWithAllNodes" type="string" value="$(arg localization)"/> 
    </node>
   
  </group>
</launch>

```

---
## Autonomous Navigation

So now... you're ready to autonomously navigate your robot around the map!  
  
In order to make the robot move, you just need to send goals to the  _/move_base/goal_  topic, as you would normally do when using the Navigation stack. If you want to send goals through Rviz, you just have to use the  _2D Nav Goal_  option as shown in the picture below:

- Note 1: Autonomous Navigation won't work until the robot localizes itself correctly. This means, until it finds a loop closure.  
- Note 2: Make sure that you are not teleoperating the robot. If you have the keyboard_teleop.launch launched, autonomous navigation may not work.

![](https://github.com/rwbot/RTAB-Map-Notes/blob/master/images/3/autonomous_navigation_final.png?raw=true)















<!--stackedit_data:
eyJoaXN0b3J5IjpbLTE5NTE0NDc2OV19
-->
