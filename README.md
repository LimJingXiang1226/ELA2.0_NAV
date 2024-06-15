# ELA2.0_NAV
ELA2.0 NAVIGATION SYSTEM WITH CARTOGRAPHER &amp; ORB-SLAM3

## Install dependencies

1) Nav2 Stack Install

	`sudo apt install ros-humble-nav2*
	sudo apt install ros-humble-navigation2`


2) Cartographer install

	`sudo apt install ros-humble-cartographer*`


3) Instruction to download ORB-SLAM3

	GO TO THE WEBSITE BELOW FOR INSTRUCTION TO INSTALL ORB-SLAM3
	
	`https://github.com/bharath5673/ORB-SLAM3`
	
 	[ READ BELOW BEFORE CONTINUING ]

	# ATTENTION 

	1) USE THIS REPO FOR ORB-SLAM3 INSTEAD OF THE ONE IN THE INSTRUCTION ABOVE
	
 		`https://github.com/zang09/ORB-SLAM3-STEREO-FIXED`
	
   	2) USE OPENCV 4.2.0

		`git checkout 4.2.0`

 	3) Troubleshooting during build

   		Add `#include <thread>` into `<path-to-opencv>/opencv/modules/gapi/test/gapi_async_test.cpp`
   
		DURING BUILDING THE ORB-SLAM3 MAY HAVE ERROR, YOU CAN DEBUG IT USING CHATGPT, YOU MAY NEED TO CHANGE FEW LINES OF CODE IN CERTAIN FILE

   	# CLONE THIS PACKAGE IN src directory
	`https://github.com/zang09/ORB_SLAM3_ROS2` [ CLONE THIS PACKAGE IN src directory ]


5) YD LiDAR INSTALLATION
	1) run command line `./YDLiDAR_SDK_INSTALLATION.sH` in directory `other`

   	2) RUN BELOW COMMAND LINE IN SRC FOLDER OF WS IF  ydlidar_ros2_driver PACKAGE IS MISSING IN SRC DIRECTORY
	
 		`git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git ydlidar_ros2_ws/src/ydlidar_ros2_driver`
	
   	3) FOR MORE INFO: https://github.com/YDLIDAR/ydlidar_ros2_driver

6) Realsense2 Library Install
	sudo apt install ros-humble-librealsense2*
	sudo apt install ros-humble-realsense2*
	[ FOR MORE INFO: https://github.com/IntelRealSense/realsense-ros]

	run command line [ "./MODIFY_realsens_launch_file.sh" ] in directory ela2_ws/README 
	Then [ COPY ] the [ rs_launch.txt file ] in the [ directory ela2_ws/README ]  to the opened python file
	[ By doing this it added some missing parameter into the Launch file ]


# ELA2.0 BRINGUP

[ Run this command line in Remote PC ]
	ssh ela2@192.168.43.125 [ IP ADDRESS MIGHT DIFFER CHECK USING "ifconfig" in Jetson Orin terminal]

[ Run this command line in ela2@ela2 terminal]
	ros2 launch ella2_bringup ella2_bringup.launch.xml

[ Run this command line in Remote PC terminal]
	[ Open New Terminal]
	ros2 launch ella2_bringup display.launch.xml


# ELA2.0 Teleop KEYBOARD

[OPEN NEW TERMINAL]
ros2 run ella2_teleop ela2_teleop_keyboard
