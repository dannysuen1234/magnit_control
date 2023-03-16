#!/usr/bin/env bash
source /etc/ubiquity/ros_setup.bash

roslaunch sick_scan sick_lms_1xx.launch &

roslaunch bluetooth_mesh bluetooth_mesh2.launch &
