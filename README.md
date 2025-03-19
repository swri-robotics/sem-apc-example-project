# Example Project

This is a simple example project and guide meant to demonstrate the pipeline for testing and getting results for the Shell Eco-marathon APC.

**Note:** This is the branch for the ROS2 Humble package. For info on the ROS1 Noetic package, navigate to the [ROS1 branch](https://github.com/swri-robotics/sem-apc-example-project).

## Requirements

[Ubuntu Linux 22.04](https://ubuntu.com/download/desktop) and [ROS2 Humble](https://docs.ros.org/en/humble/)

Use the [APC Docker environment](https://github.com/swri-robotics/sem-apc-student-docker-environment) to set up ROS and CARLA or set up [ROS](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) and [CARLA](https://carla.readthedocs.io/en/0.9.15/start_quickstart/) locally.

Important considerations for the competition:
* Project submissions are compiled in a fresh Linux and ROS environment.
* Any software dependencies must be defined properly in the [catkin package manifest](http://wiki.ros.org/catkin/package.xml) and will be installed at build time by [rosdep](http://wiki.ros.org/rosdep).
* All ROS packages must [install](http://wiki.ros.org/catkin/CMakeLists.txt#Optional_Step:_Specifying_Installable_Targets) themselves when built; the source code will not be present in the simulation environment, only installed targets.
* All project submissions ***must*** have a package named `shell_simulation` with a launch file named `shell_simulation.launch.py` that requires no parameters or arguments in order to run; this is used as the entry point for launching the project.
* Uploaded projects ***must*** be named `project.zip` and contain only the source code of ROS packages.
* The simulation will automatically end after either all goals have been reached or after it has been running for 10 minutes.

The map and goal points will be revealed before the start of the competition.

## Running the Example

After building your [APC Docker environment](https://github.com/swri-robotics/sem-apc-student-docker-environment) (recommended) or installing and configuring ROS and CARLA locally, you should now be able to develop, build, and run your code to control the vehicle in simulation. Let's run an example that moves the vehicle forward to show you how to build and run your code to control the vehicle.

1. If you are using Docker, make sure the carla_server and ros_environment containers are running and that you have entered your ros_environment container *(If you are running ROS and CARLA locally, startup CARLA and skip this step.)*:

    `docker start carla_server ros_environment`

    `docker exec -it ros_environment /bin/bash`

2. Navigate to your ROS workspace directory:

    `cd shell_ws`

3. Build your ROS workspace:

    `colcon build`

4. Source your workspace:

    `source install/setup.bash`

5. Launch the carla_shell_bridge interface to setup the server world and spawn a vehicle to control:

    `ros2 launch carla_shell_bridge main.launch.py`

    This should open up an Rviz window displaying vehicle sensor data.

    There are also a variety of parameters within the [carla-interface](https://github.com/swri-robotics/sem-apc-carla-interface/tree/ros2) package that can be modified to configure the simulation and perform tasks such as selecting a different map, setting a spawn point, and generating traffic. To change these parameters, modify the values in the `carla_config.yaml` file located in your ROS workspace: `~/shell_ws/src/carla-interface/config/carla_config.yaml`

    *Note, you will need to relaunch the carla_shell_bridge for these changes to take effect.*

6. Finally, open a new terminal, enter your ros_environment container and ROS workspace as shown in steps 1 and 2 *(skip step 1 if you are running ROS locally)*, and source your workspace as shown in step 4. You can now run either the C++ or Python example node that moves the vehicle forward!
    
    `ros2 run shell_simulation example_control`
    
    or

    `ros2 run shell_simulation example_control.py`

## General Tips
- Since your workspace is mounted to the ros_environment Docker container, you can simply edit your code locally in your ROS workspace with your favorite text editor, and all the changes will be synced to the Docker container automatically.

- While building your ROS packages, you can use the following command to only build a specific package which can greatly reduce build time:

  `colcon build --packages-select <YOUR_PACKAGE_NAME>`

- If you encounter a spawning error with the ego vehicle (`Exception caught: Spawn failed because of collision at spawn position`) you may need to change the spawn point parameter in the `carla_config.yaml` file. The Z coordinate may need to be set to something greater than 0. You can also change this parameter to "None" which will spawn the vehicle in a random, valid position on the map.

## Topics

The following ROS topics are available within the simulation:

```
Published topics:
  * /carla/ego_vehicle/collision [carla_msgs/CarlaCollisionEvent]
  * /carla/ego_vehicle/depth_middle/image [sensor_msgs/Image]
  * /carla/ego_vehicle/depth_middle/camera_info [sensor_msgs/CameraInfo]
  * /carla/ego_vehicle/gnss [sensor_msgs/NavSatFix]
  * /carla/ego_vehicle/imu [sensor_msgs/Imu]
  * /carla/ego_vehicle/lane_invasion [carla_msgs/CarlaLaneInvasionEvent]
  * /carla/ego_vehicle/odometry [nav_msgs/Odometry]
  * /carla/ego_vehicle/speedometer [std_msgs/Float32]
  * /carla/ego_vehicle/rgb_front [sensor_imgs/Image]
  * /carla/ego_vehicle/vehicle_status [carla_msgs/CarlaEgoVehicleStatus]
  * /carla/ego_vehicle/vlp16_1 [sensor_msgs/PointCloud2]
  * /clock [rosgraph_msgs/Clock]
  * /rosout [rosgraph_msgs/Log] 5 publishers
  * /rosout_agg [rosgraph_msgs/Log]
  * /tf [tf2_msgs/TFMessage]

  These topics have data from sensors that can be used to observe the environment:
  * /carla/ego_vehicle/depth_middle/image [sensor_msgs/Image]
  * /carla/ego_vehicle/depth_middle/camera_info [sensor_msgs/CameraInfo]
  * /carla/ego_vehicle/gnss [sensor_msgs/NavSatFix]
  * /carla/ego_vehicle/lane_invasion [carla_msgs/CarlaLaneInvasionEvent]
  * /carla/ego_vehicle/imu [sensor_msgs/Imu]
  * /carla/ego_vehicle/odometry [nav_msgs/Odometry]
  * /carla/ego_vehicle/vlp16_1 [sensor_msgs/PointCloud2]
```

And messages can be published to this topic to control the vehicle:

```
  *  /brake_command [std_msgs/Float64]
  Valid values range from 0.0 (no brake) to 1.0 (full brake)
  *  /gear_command [std_msgs/String]
  Valid values are "forward" or "reverse"
  *  /handbrake_command [std_msgs/Bool]
  If set to "true", throttle will be ignored
  *  /steering_command [std_msgs/Float64]
  Valid values range from -1.0 (full left) to 1.0 (full right)
  *  /throttle_command [std_msgs/Float64]
  Valid values range from 0.0 (no throttle) to 1.0 (full throttle)
```
