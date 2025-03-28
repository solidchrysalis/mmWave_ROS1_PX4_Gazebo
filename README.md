# mmWave_ROS1_PX4_Gazebo
Simulate mmWave radar based drone control in Gazebo with ROS1 and PX4 1.13

### Prerequisites
Tested with:
- Ubuntu 20.04.3 LTS
- ROS1 Noetic
- Gazebo 11.9.0
- PX4 Autopilot commit d7a962b4269d3ca3d2dcae44da7a37177af1d8cd

Specific commits from [pull request #1](https://github.com/nhma20/mmWave_ROS2_PX4_Gazebo/pull/1/commits/3ad006353495266432122e5ec3f595899fedeb08)


### Install ROS2
https://docs.ros.org/en/foxy/Installation.html or https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html
- Setup Sources:
```sh
sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```
- Install ROS 1 packages:
```sh
sudo apt update
sudo apt install ros-noetic-desktop
```


### Install Gazebo
http://gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros and http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install
- Default Gazebo installation:
```sh
cd ~
curl -sSL http://get.gazebosim.org | sh
```
- Install gazebo_ros_pkgs
```sh
sudo apt install ros-noetic-gazebo-ros-pkgs
```
- Install gazebo MAVROS packages
```sh
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-mavlink 
```
- Install GeographicLib
```sh
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh   
```

### Install PX4 (ROS, SITL, Gazebo)
- Download PX4 Source code, change to ROS1 and run ```ubuntu.sh``` with no arguments:
```sh
cd ~
git clone -n https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot/
git checkout release/1.13
git submodule update --init --recursive
bash ./Tools/setup/ubuntu.sh
```
- Relogin or reboot computer before attempting to build NuttX targets
- Setup ROS 1 Workspace (**if needed**)
```sh
cd ~
source /opt/ros/noetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ..
catkin_make
source /devel/setup.bash

```

### Modify Drone to Include Lidar Plugin

- Enable distance sensor in mavros - comment out distance sensor and rangefinder
```sh
sudo vim /opt/ros/noetic/mavros/launch/px4_pluginlists.yaml
```
- Add LaserScan plugin to lidar in gazebo
```sh
cd (PX4_DIR)/Tools/sitl_gazebo/models/lidar
vi model.sdf

# Add underneath LaserPlugin
<plugin name="TrueLaser" filename="libgazebo_ros_laser.so">
     <robotNamespace></robotNamespace>
          <topicName>/laser/scan</topicName>
          <frameName>/lidar_sensor_link</frameName>
</plugin>

```

### Install Python Packages for Utils

```sh
sudo pip3 install pypcd4
```

### Install QGroundControl
https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html#ubuntu
```sh
sudo usermod -a -G dialout $USER
sudo apt-get remove modemmanager -y
sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y
```
- Download: https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage
```sh
cd ~/Downloads/
chmod +x ./QGroundControl.AppImage
./QGroundControl.AppImage  (or double click)
   ```


### Test if all works
(https://docs.px4.io/master/en/ros/ros2_comm.html#sanity-check-the-installation)
1. Open a new terminal in the root of the PX4 Autopilot project:
   ```sh
   cd ~/PX4-Autopilot/
   ```
   
   (syntax: ```make <target> <simulator>_<vehiclemodel>__<world> ```
   prepend ```HEADLESS=1``` to launch without GUI
   prepend ```PX4_NO_FOLLOW_MODE=1``` to launch without following drone)

   Should make and open PX4 in same console, as well as a Gazebo window with chosen model and world
  
2. On a new terminal, start MAVROS:
   ```sh 
   source /opt/ros/noetic/setup.bash
   roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
   ```
  
3. On the original terminal start the PX4 Simulation:
   ```sh
   DONT_RUN=1 make px4_sitl_default gazebo-classic
   source ~/catkin_ws/devel/setup.bash    # (optional)
   source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
   export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
   export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic
   roslaunch px4 posix_sitl.launch
   ```
  
4. Open a new terminal and start a mmWave converter using the provided launch file:
   ```sh 
   source /opt/ros/noetic/setup.bash
   source ~/catkin_ws/devel/setup.bash
   roslaunch mmWave_ROS1_PX4_Gazebo main.launch
   ```
   
5. Optionally, open QGroundControl which will connect with PX4. From here it is possible to set waypoints and execute missions.


### Launch all
https://docs.px4.io/master/en/ros/ros2_offboard_control.html
https://github.com/PX4/px4_ros_com/blob/master/src/examples/offboard/offboard_control.cpp

0. If offboard_control.cpp or other files have been edited, re-run ```install.sh``` script (add new files to script and CMakeLists.txt):
   ```sh
   cd ~/mmWave_ROS2_PX4_Gazebo/
   ( chmod +x ./install.sh )
   ./install.sh
   ```
    If same PX4 and px4_ros_com_ros2 roots:
    ```
    ./install.sh ~/PX4-Autopilot/ ~/px4_ros_com_ros2/
    ```
1. Launch PX4 SITL:
   ```sh
    cd ~/PX4-Autopilot/ 
    make px4_sitl_rtps gazebo_iris__hca_full_pylon_setup
   ```
   Without Gazebo GUI:
   ```sh
    HEADLESS=1 make px4_sitl_rtps gazebo_iris__hca_full_pylon_setup
   ```
   Without drone following:
   ```sh
    PX4_NO_FOLLOW_MODE=1 make px4_sitl_rtps gazebo_iris__hca_full_pylon_setup
   ```
   After PX4 SITL fully launched, might need to manually start microRTPS client in same terminal:
   ```sh
    micrortps_client start -t UDP
   ```
   Will fail and return -1 if already running.
2. Open QGroundControl   
3. In a new terminal start microRTPS agent and offboard control:
   ```sh 
   source ~/px4_ros_com_ros2/install/setup.bash
   micrortps_agent start -t UDP & ros2 run px4_ros_com offboard_control 
   ```
4. In another terminal, start the velocity vector advertiser, lidar to mmwave converter, and 3d to 2d projection nodes:
   ```sh 
   source ~/px4_ros_com_ros2/install/setup.bash
   ros2 launch ~/mmWave_ROS2_PX4_Gazebo/launch/simulate_pointcloud_control_launch.py 
   ```
5. Simulated drone in Gazebo should arm and takeoff. May need to restart ```vel_ctrl_vec_pub``` and ```offboard_control``` ros2 runs.

6. Visualize simulated data in rviz2:
   ```sh 
   rviz2 ~/mmWave_ROS2_PX4_Gazebo/3d_and_2d_pointcloud_rgb.rviz 
   ```


### Adding wind to the simulation
To enhance the realism of the simuation, it is possible to add wind to the virtual environment. This is simply done by adding and customizing the wind plugin in the .world file. Below is an example which can be added in the `hca_full_pylon_setup.world` file which will introduce a mean wind of 3m/s, a max wind velocity of 6m/s, and a typical wind direction along the y-axis:

```sh
	<plugin name='wind_plugin' filename='libgazebo_wind_plugin.so'>
      <frameId>base_link</frameId>
      <robotNamespace/>
      <windVelocityMean>3.0</windVelocityMean>
      <windVelocityMax>6.0</windVelocityMax>
      <windVelocityVariance>0.25</windVelocityVariance>
      <windDirectionMean>0 1 0</windDirectionMean>
      <windDirectionVariance>0.25</windDirectionVariance>
      <windGustStart>0</windGustStart>
      <windGustDuration>0</windGustDuration>
      <windGustVelocityMean>0</windGustVelocityMean>
      <windGustVelocityMax>20.0</windGustVelocityMax>
      <windGustVelocityVariance>0</windGustVelocityVariance>
      <windGustDirectionMean>1 0 0</windGustDirectionMean>
      <windGustDirectionVariance>0</windGustDirectionVariance>
      <windPubTopic>world_wind</windPubTopic>
    </plugin>
```

### MISC
0. General tips on PX4+Gazebo simulation (e.g. wind, vehicle spawn location): https://docs.px4.io/main/en/simulation/gazebo.html
1. Trajectory setpoint message:
   https://github.com/PX4/px4_msgs/blob/ros2/msg/TrajectorySetpoint.msg
2. Changed parameters (to fix "Failsafe enabled: No manual control stick input" warning and not taking off):
   pxh> param set NAV_RCL_ACT 0
   pxh> param set COM_RCL_EXCEPT 4

   NAV_RCL_ACT: curr: 2 -> new: 0
3. Local positioning?
   https://github.com/PX4/px4_msgs/blob/ros2/msg/VehicleLocalPositionSetpoint.msg
   No, calculate positions in drone frame and transform to world frame.
   
4. Add any new ROS2 files to ~/px4_ros_com_ros2/src/px4_ros_com/CMakeLists.txt

5. Check if drone armed? https://github.com/PX4/px4_msgs/blob/ros2/msg/ActuatorArmed.msg
   No, subscribe to `/fmu/vehicle_status/out` topic and monitor `arming_state`.

6. libignition-common3 error (after software update?) - Copy existing file and rename to match missing file
7. If gazebo does not open, try running ```gazebo --verbose``` to troubleshoot. ```killall gzserver``` should kill any gazebo instances. Restart PC if all else fails.
8. inlude both iris.sdf and iris.sdf.jinja?
9. Implemented laser scanner with Gazebo and ROS2 https://github.com/chapulina/dolly
10. Make custom sensor plugin http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i5
11. In ~/px4_ros_com_ros2/src/px4_ros_com/CMakeLists.txt add ```sensor_msgs``` under ```ament_target_dependencies```
12. After running ```./build_ros2_workspace``` restart all affected executables (micrortps_agent, offboard_control, vel_vec_ctrl_pub). Gazebo PX4 SITL can be left running.
13. iris.sdf (or other models) can be edited to include sensors, like 2D lidar.
14. Display simulated camera feed either with rviz2 or
   ```sh 
   source ~/px4_ros_com_ros2/install/setup.bash
   ros2 run image_tools showimage image:=/cable_camera/image_raw
   ```
14. Add new worlds/models to ~/PX4-Autopilot/platforms/posix/cmake/sitl_target.cmake
15. See local packages, and msgs, with: ```ros2 interface packages``` and e.g. ```ros2 interface package px4_msgs```
16. Camera intrinsic parameters for setting a custom perspective projection matrix (cannot be used with WideAngleCamera since this class uses image stitching from 6 different cameras for achieving a wide field of view). The focal lengths can be computed using focal_length_in_pixels = (image_width_in_pixels * 0.5) / tan(field_of_view_in_degrees * 0.5 * PI/180) (http://sdformat.org/spec?ver=1.7&elem=sensor#lens_intrinsics)
17. Drone spawn coordinates set in ~/PX4-Autopilot/Tools/sitl_run.sh ?
18. ```*** No rule to make target '/opt/ros/foxy/lib/libfastrtps.so.2.0.2', needed by 'libpx4_msgs__rosidl_typesupport_fastrtps_cpp.so'.  Stop.```
Fixed by renaming closest libfastrtps.so.x.y.z to libfastrtps.so.2.0.2.
19. Dependency errors with PX4, like ```ninja: error: '/usr/lib/x86_64-linux-gnu/libsdformat9.so.9.6.1', needed by 'libmav_msgs.so', missing and no known rule to make it``` may be solved by a PX4 reinstall (remember worlds, models, cmake files etc. must be also be reinstalled into new PX4).
20. If drone enters failsafe when starting offboard_control, ```param set COM_RCL_EXCEPT 4``` in the PX4 console may solve this. Else, try manually publish few setpoints to fmu/manual_control_setpoint/in and then start offboard mode.
21. Showing videos in readme: Just drag and drop your image/video from your local pc to github readme in editable mode.
22. If gradle not working, might have to downgrade Java (JDK) to 11: https://askubuntu.com/questions/1133216/downgrading-java-11-to-java-8
23. May have to set unused non-velocity parameters to NAN in TrajectorySetpoint message: https://discuss.px4.io/t/offboard-control-using-ros2-how-to-achieve-velocity-control/21875
24. Customize GPS noise within .sdf of vehicle model in the gps_plugin section. E.g. gpsXYRandomWalk of 0.02 and gpsZRandomWalk of 0.04 if simulating RTK accuracy.




### TODO
0. :green_circle: Install tools 
1. :green_circle: Figure out how to control drone via offboard_control.cpp 
2. :green_circle: Make ROS2 advertiser that generates control input for offboard_control.cpp for more advanced control
3. :green_circle: Figure out how to use simulated depth sensors
4. :green_circle: Implement depth data into ROS2 advertiser for even more advanced control
5. :green_circle: Control drone towards overhead cable

![Alt text](https://github.com/nhma20/mmWave_ROS2_PX4_Gazebo/blob/main/Pictures/Screenshot_from_2021-06-08_15-17-35.png?raw=true)

7. :yellow_circle: More tightly integrate with PX4 to optimize control based on e.g. drone state
   - get pose of drone to mitigate sideways motion when rotated around x or y.
   - use GPS positioning to counteract drift
9. :green_circle: Use drone mounted simulated camera to get images of overhead cable 
10. :green_circle: Visualize depth data in camera feed

![Alt text](https://github.com/nhma20/mmWave_ROS2_PX4_Gazebo/blob/main/Pictures/Screenshot_from_2021-07-05_20-52-58.png?raw=true)

12. :green_circle: Investigate occasional drone control loss
13. :green_circle: Make module that turns 2d lidar data into noisy pointcloud to prepare for mmwave integration
14. :yellow_circle: Tracking of points in pointcloud (kalman?)
15. :yellow_circle: Implement cable detection AI to filter depth data and align drone yaw wrt. cable

![Alt text](https://github.com/nhma20/mmWave_ROS2_PX4_Gazebo/blob/main/Pictures/Screenshot_from_2021-07-06_09-54-53.png?raw=true)

https://user-images.githubusercontent.com/76950970/142616665-0cb08003-9355-4eed-a658-7d900c9f66fb.mp4




