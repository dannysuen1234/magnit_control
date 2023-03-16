#trap 'pkill -f start' INT
#!/usr/bin/env bash
source /etc/ubiquity/ros_setup.bash

roslaunch magni_lidar start_planner.launch &
roslaunch magni_lidar amcl.launch 
