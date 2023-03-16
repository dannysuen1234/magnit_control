#!/usr/bin/env python3

#include the libraries here
import rospy
import rospkg
import threading
from bluetooth_mesh.msg import Message
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

#publisher function
def publish_bluetooth(dest, source, data):
	the_message = Message()
	the_message.msg.dest = dest
	the_message.msg.source = source
	the_message.msg.data = data
	pub = rospy.Publisher('/bluetooth_mesh/send', Message, queue_size = 10)
	rospy.sleep(1)
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
	rospy.init_node("testing")

	
	move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

	#go to first point
	go_to(3.5, -1, -1.57)

	
	#send bluetooth message to open door
	#code if using thread
	#publish_thread = threading.Thread (target = publish_bluetooth, args = [0x830, 0x82, [0xc4, 0x00, 0xa0, 0x00, 0x80]])
	#publish_thread.start()

	#send bluetooth message to open door
	subscribe_till_response(0x830, 0x82, [0xc4, 0x00, 0xa0, 0x00, 0x80], "/bluetooth_mesh/receive", 2, [0xc6, 0x00, 0xa0, 0x00, 0x80])
	

	#check if door is being opened
	subscribe_till_response(0x830, 0x82, [0xc1, 0x00, 0xa0, 0x00, 0x80], "/bluetooth_mesh/receive", 2, [0xc2, 0x00, 0xa0, 0x00, 0x80])


	#subscribe_thread = threading.Thread(target = subscribe_till_response, args = ["/bluetooth_mesh/receive", 5, [0xc2, 0x00, 0xa0, 0x00, 0x80]] )
	#subscribe_thread.start()

	#while not sent_open_door:
	#	publish_bluetooth(0x830, 0x82, [0xc1, 0x00, 0xa0, 0x00, 0x80])
	#	rospy.sleep(1)

	#move to next goal
	go_to(3.5, -7.5, -1.57)

	#close door

