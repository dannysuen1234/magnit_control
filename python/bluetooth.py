#!/usr/bin/env python3

#include the libraries here
import rospy
import rospkg
import threading
from bluetooth_mesh.msg import Message
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
import time
import os
from geometry_msgs.msg import PoseWithCovarianceStamped
import threading
from paho.mqtt import client as paho
import json

def compare_list(data, expected):
	for i in range(len(expected)):
		if (data[i]!=expected[i]):
			return False
	return True
def publish_bluetooth(dest, source, data):

	the_message = Message()
	the_message.msg.dest = dest
	the_message.msg.source = source
	the_message.msg.data = data
	pub = rospy.Publisher('/bluetooth_mesh/send', Message, queue_size = 10)
	pub.publish(the_message)
	print("pubed", dest, data)

def wait_lift(bluetooth_dest, bluetooth_source, bluetooth_data, topic, seconds_to_retry, expected_message):
	global lift_addr
	while lift_addr is None:
		try:

			
			publish_bluetooth(bluetooth_dest, bluetooth_source, bluetooth_data)
			rospy.sleep(seconds_to_retry)
			received = rospy.wait_for_message(topic, Message, timeout=seconds_to_retry)
			print('received data =', received, 'expected = ', expected_message)
			if (compare_list(received.msg.data, expected_message) == True):
				lift_addr = received.msg.source
				return

		except Exception as e:
			print(e)

lift_addr = None

if __name__ == "__main__":
	rospy.init_node("testing", anonymous = True)
	wait_lift(0x430, 0x82, [0xc1, 0x00, 0xa0, 0x01, 0x01], "/bluetooth_mesh/receive", 0.5, [0xC2, 0x00, 0xA0, 0x01, 0x01])
