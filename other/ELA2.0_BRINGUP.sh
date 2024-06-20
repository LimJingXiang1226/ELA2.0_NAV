#!/bin/bash

# Set the DISPLAY environment variable
export DISPLAY=:0

# Open a new terminal with the ROS 2 launch command
x-terminal-emulator -e "bash -c 'source /home/ela2/.bashrc && source /opt/ros/humble/setup.bash && source /home/ela2/ella2_ws/install/setup.bash && export ROS_DOMAIN_ID=2 && ros2 launch ella2_bringup ella2_bringup.launch.xml; exec bash'"
