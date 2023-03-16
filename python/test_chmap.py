#!/usr/bin/env python3

import rospkg
import rospy
import os
from geometry_msgs.msg import PoseWithCovarianceStamped
import threading
rospy.init_node("test")
pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size =10)
from tf.transformations import quaternion_from_euler

q = quaternion_from_euler(0, 0, 1.57)
	
print(q)



pose = PoseWithCovarianceStamped()
pose.header.frame_id = "map"
pose.pose.pose.position.x = 3
pose.pose.pose.position.y = 0
pose.pose.pose.orientation.w = 1

rospack = rospkg.RosPack()
map_path = rospack.get_path("magni_lidar") + "/maps/3f_2lift.yaml"
change_map_thread = threading.Thread(target = os.system, args=["rosrun map_server map_server " + map_path])
#change_map_thread.start()


#pub.publish(pose)
command = '''rostopic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped "header: 
  seq: 
  stamp: now
  frame_id: "map"
pose: 
  pose: 
    position: 
      x: 0
      y: 0
      z: 0.0
    orientation: 
      x: 0.0
      y: 0.0
      z: 0.731581028249
      w: 0.681754500613
  covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]"'''

os.system(command)
