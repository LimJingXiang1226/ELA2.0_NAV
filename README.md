# ELA2.0_NAV

ELA2.0 NAVIGATION SYSTEM WITH CARTOGRAPHER &amp; ORB-SLAM3

<p align="center">
  <img src="https://github.com/LimJingXiang1226/ELA2.0_NAV/raw/main/other/image/ELA2.0(2).jpeg" alt="ELA2.0" />
</p>

System Tested on below version
- Ubuntu 22.04
- ROS2 HUMBLE
- OpenCV 4.2.0
- YD LiDAR X2
- Intel Realsense D435i 

## Install dependencies

### Nav2 Stack Install

`sudo apt install ros-humble-nav2*`

`sudo apt install ros-humble-navigation2`

### Cartographer Install

`sudo apt install ros-humble-cartographer*`


### Instruction to Install ORB-SLAM3

1) GO TO THE GITHUB REPO BELOW FOR INSTRUCTION TO INSTALL ORB-SLAM3

	`https://github.com/bharath5673/ORB-SLAM3`
	
	[ READ BELOW BEFORE CONTINUING ]

2) ATTENTION 

	1) USE THIS REPO FOR ORB-SLAM3 INSTEAD OF THE ONE IN THE INSTRUCTION ABOVE
	
		`https://github.com/zang09/ORB-SLAM3-STEREO-FIXED`
	
	2) USE OPENCV 4.2.0

		`git checkout 4.2.0`

	3) Troubleshooting during build

		Add `#include <thread>` into `<path-to-opencv>/opencv/modules/gapi/test/gapi_async_test.cpp`
   
		DURING BUILDING THE OPENCV 4.2.0 MAY HAVE ERROR, YOU MAY NEED TO DEBUG IT AND CHANGE FEW LINES OF CODE IN CERTAIN FILE

3) CLONE THIS PACKAGE IN SRC DIRECTORY
	
 	`https://github.com/zang09/ORB_SLAM3_ROS2`



### YD LiDAR INSTALLATION

1) run command line `./YDLiDAR_SDK_INSTALLATION.sh` in directory `other`

2) RUN BELOW COMMAND LINE IN SRC FOLDER OF WS IF  ydlidar_ros2_driver PACKAGE IS MISSING IN SRC DIRECTORY

	`git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git ydlidar_ros2_ws/src/ydlidar_ros2_driver`

3) FOR MORE INFO: https://github.com/YDLIDAR/ydlidar_ros2_driver

### Realsense2 Library Install
	
1) RUN BELOW COMMAND LINE

	`sudo apt install ros-humble-librealsense2*`
	
	`sudo apt install ros-humble-realsense2*`

2) FOR MORE INFO: https://github.com/IntelRealSense/realsense-ros

3) MODIFICATION TO BE MADE TO DEFAULT LAUNCH FILE

	run command line `./MODIFY_realsens_launch_file.sh` in `other` directory
	
	Then `COPY` the `rs_launch.txt` file in the `other` directory  to the opened python file

	By doing this it added some missing parameter into the Launch file


### Laser Odometry Install

1) CLONE THIS PACKAGE IN SRC DIRECTORY

	`https://github.com/MAPIRlab/rf2o_laser_odometry.git`

### Mecanum Drive Control

1) CLONE THIS PACKAGE IN SRC DIRECTORY
   
	`https://github.com/deborggraever/ros2-mecanum-bot.git`

2) Modification made to the package

   Add `ella2_hardware.py` file in the Launch Folder of `mecanumbot_bringup` package

   [`ella2_hardware.py` is in `other` folder in this repo]

### Robot Pose Package
1) CLONE THIS PACKAGE IN SRC DIRECTORY

   `git clone https://github.com/MilanMichael/robot_pose_publisher_ros2`

3) Modify `robot_pose_publisher.cpp` in src directory of package

   Include the Neccesary Header: `#include <tf2_ros/buffer.h>` and `#include <tf2_ros/transform_listener.h>`

### tf_transformation Library for Pose_Fusion

1) Run Following CommandLine:

   `git clone https://github.com/DLu/tf_transformations.git`

   `cd tf_transformations/`

   `pip install .`

## ELA2.0 BRINGUP

### Setup Jetson Orin Nano to Automatically run ELA2.0 BRINGUP (Physical Robot)

1) Navigate to `other` directory 

2) Run following command line

   `./Jetson_AutoBringup_setup.sh`

3) If user name is not `ela2`:

   Replace User Name in `ELA2.0_BRINGUP.sh` [Home Directory] & `ela2.service` [ /etc/systemd/system ]

   Run `sudo systemctl daemon-reload && sudo systemctl enable ela2.service`
   

5) If Issue oOccur, Debug Using Following Commandline:
   
   `sudo systemctl status ela2.service`

   `journalctl -u ela2.service -f`

### Without Auto ELA2.0 BRINGUP (Physical Robot)

1) Run this command line in Remote PC

 	run `ssh ela2@IP_ADDRESS`

 	[ IP ADDRESS MIGHT DIFFER CHECK USING "ifconfig" in Jetson Orin terminal]

2) Run this command line in ela2@ela2 terminal [ELA2.0 BRINGUP]

	`ros2 launch ella2_bringup ella2_bringup.launch.xml`

### Simulation Bringup
1) Run this Following line

   `ros2 launch ella2_bringup ella2_bringup.launch.xml simulation:=true use_sim_time:=true`
   
## ELA2.0 Teleop KEYBOARD

1) OPEN NEW TERMINAL
2) run `ros2 run ella2_teleop ela2_teleop_keyboard`
