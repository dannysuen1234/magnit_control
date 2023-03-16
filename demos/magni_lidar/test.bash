trap 'echo '123'' INT
#!/usr/bin/env bash
source /etc/ubiquity/ros_setup.bash


roslaunch magni_lidar amcl.launch &
roslaunch magni_lidar start_planner.launch
