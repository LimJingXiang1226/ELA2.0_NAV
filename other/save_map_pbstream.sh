#!/bin/bash

# Check if FILENAME environment variable is set
if [ -z "$MAP_FILENAME" ]; then
    echo "Please set the MAP_FILENAME environment variable."
    echo "Usage: export MAP_FILENAME=<filename> && ./save_map.sh"
    exit 1
fi

cd
ros2 run nav2_map_server map_saver_cli -f ~/ELA2.0_NAV/install/ella2_nav/share/ella2_nav/maps/${MAP_FILENAME}
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '${MAP_FILENAME}.pbstream'}" && cp ~/${MAP_FILENAME}.pbstream ~/ELA2.0_NAV/src/ella2_nav/maps && mv ~/${MAP_FILENAME}.pbstream ~/ELA2.0_NAV/install/ella2_nav/share/ella2_nav/maps
