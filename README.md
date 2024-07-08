# rtab_sim: Indoor Mapping Drone Simulation Package

## Pre-requisites

1. Ubuntu 20.04 with ROS Noetic installation
2. Gazebo 11 Installation
3. ArduPilot SITL Installation
4. ArduPilot Gazebo Plugins Installation

## Package Build

To build the package: 

1. Create a workspace folder in your ```$HOME``` directory:
```bash
mkdir -p <your_ws>/src
```
Replace ```your_ws``` with the name of your workspace.

2. Clone the rtab_sim ROS Package:
```bash
cd <your_ws>/src
git clone https://github.com/fabeha-raheel/rtab_sim.git
```

3. Build the ROS Package:
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

