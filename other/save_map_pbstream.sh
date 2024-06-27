cd
ros2 run nav2_map_server map_saver_cli -f ~/ELA2.0_NAV/install/ella2_nav/share/ella2_nav/maps/apcore2
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: 'apcore2.pbstream'}" && mv ~/apcore2.pbstream ~/ELA2.0_NAV/install/ella2_nav/share/ella2_nav/maps

