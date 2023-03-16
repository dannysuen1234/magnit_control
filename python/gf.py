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


#publisher function
def publish_bluetooth(dest, source, data):
	the_message = Message()
	the_message.msg.dest = dest
	the_message.msg.source = source
	the_message.msg.data = data
	pub = rospy.Publisher('/bluetooth_mesh/send', Message, queue_size = 10)
	rospy.sleep(0.5)
	pub.publish(the_message)
	print("pubed")
		
		
#subscriber function (subscribe continuously until there is a response)
def subscribe_till_response(bluetooth_dest, bluetooth_source, bluetooth_data, topic, seconds_to_retry, expected_message):
	while True:
		try:
			publish_bluetooth(bluetooth_dest, bluetooth_source, bluetooth_data)
			print("done 1")
			received = rospy.wait_for_message(topic, Message, timeout=seconds_to_retry)
			print("2")
			print('received data =', received, 'expected = ', expected_message)
			if (compare_list(received.msg.data, expected_message) == True):
				return

		except:
			print("not yet")
		

def subscribe_for_second(topic, expected_message, second):
	try:
		received = rospy.wait_for_message(topic, Message, timeout=second)
		if (compare_list(received.msg.data, expected_message) == True):
			return True
		else:
			return False

	except:
		return False

def press_lift_button(dest, source, data, sub_topic, expected_message):
	button_pressed = False
	while not button_pressed:
		print("life button not pressed yet!")
		publish_bluetooth(dest, source, data)
		button_pressed = subscribe_for_second(sub_topic, expected_message, 1)
	print("lift button pressed")

#function for going to next goal
def go_to(x, y, z):
	move_base_client.wait_for_server()
	goal = MoveBaseGoal()
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.header.stamp = rospy.Time.now()
	goal.target_pose.pose.position.x = x
	goal.target_pose.pose.position.y = y
	#define pose here
	q = quaternion_from_euler(0, 0, z)
	
	goal.target_pose.pose.orientation.x = q[0]
	goal.target_pose.pose.orientation.y = q[1]
	goal.target_pose.pose.orientation.z = q[2]	
	goal.target_pose.pose.orientation.w = q[3]
	
	move_base_client.send_goal(goal)

	wait = move_base_client.wait_for_result()

#function to check two lists are the same
def compare_list(data, expected):
	for i in range(len(expected)):
		if (data[i]!=expected[i]):
			return False
	return True



if __name__ == "__main__":
	rospy.init_node("testing2", anonymous = True)
	
	
	
	pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size =10)

	move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)



	#go to the door
	go_to(2, -4.8, 1.57)

	#open door
	press_lift_button(0x800, 0x82, [0xC4, 0x00, 0xA0, 0x00, 0x80], "/bluetooth_mesh/receive", [0xC6, 0x00, 0xA0, 0x00, 0x80])
	print("pressed")
	rospy.sleep(2)
	go_to(0, 0, 0)
	
