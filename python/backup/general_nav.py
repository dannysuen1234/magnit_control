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
# function to create change robot pose command
def change_pose_command(x, y, theta):
	q = quaternion_from_euler(0, 0, theta)
	command = '''rostopic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped "header: 
  seq: 
  stamp: now
  frame_id: "map"
pose: 
  pose: 
    position: 
      x: ''' +str(x)+ '''
      y: ''' + str(y)+''' 
      z: 0.0
    orientation: 
      x: ''' + str(q[0]) +'''
      y: ''' + str(q[1]) +'''
      z: ''' + str(q[2]) +'''
      w: ''' + str(q[3]) + ''' 
  covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]"'''

	return command

# function to go from a floor to another floor
curr_floor = 3
curr_x = 0
curr_y = 0
def change_floor(target_floor):
	global curr_floor, curr_x, curr_y
	#add target floor command in hex 

	map_name_dict = {0: "gf_2lift_2", 3: "3f_2lift"}

	lobby_addr_dict = {0: 0x400, 1: 0x410, 2: 0x420, 3: 0x430}
	lift_addr = 0x330

	lobby_addr = lobby_addr_dict(curr_floor)
	target_lobby_addr = lobby_addr_dict(target_floor)

	curr_floor_lift_location = {0 : [0, 0, 0], 3 : [0.45, -0.5, -1.57]}
	curr_floor_inside_lift_location={0:[0, 0, 0], 3:[0.45, -3, 1.57]}
	target_floor_inside_lift_location = {0 : [-1.35, -11, 1.57], 3 : [0.45, -3, 1.57]}

	curr_floor_nav_point = curr_floor_lift_location[curr_floor]
	curr_floor_lift_nav_point = curr_floor_inside_lift_location[curr_floor]
	target_floor_nav_point = target_floor_inside_lift_location[target_floor]
	same_floor_nav(curr_floor_nav_point[0], curr_floor_nav_point[1], curr_floor_nav_point[2])
	curr_x = curr_floor_nav_point[0]
	curr_y = curr_floor_nav_point[1]

	#send bluetooth message to press lift button, wait for response
	#send bluetooth message to open door
	press_lift_button(lobby_addr, 0x82, [0xC4, 0x00, 0xA0, 0x02, 0x00], "/bluetooth_mesh/receive", [0xC6, 0x00, 0xA0, 0x02, 0x00])
	print("lift button pressed")

	#keep checking if lift is arrived and lift door opened
	subscribe_till_response(lobby_addr, 0x82, [0xc1, 0x00, 0xa0, 0x01, 0x01], "/bluetooth_mesh/receive", 0.5, [0xC2, 0x00, 0xA0, 0x01, 0x01])
	print("lift arrived")

	#step 4: hold the life, then go into the lift
	subscribe_till_response(lobby_addr, 0x82, [0xC4, 0x00, 0xA0, 0x00, 0x80], "/bluetooth_mesh/receive", 2, [0xC6, 0x00, 0xA0, 0x00, 0x80])
	go_to(curr_floor_lift_nav_point[0], curr_floor_lift_nav_point[1], curr_floor_lift_nav_point[2])
	curr_x = curr_floor_lift_nav_point[0]
	curr_y = curr_floor_lift_nav_point[1]
	print("entered lift")

	#step 5: release the open lift door command
	subscribe_till_response(lobby_addr, 0x82, [0xC4, 0x00, 0xA0, 0x00, 0x40], "/bluetooth_mesh/receive", 0.1, [0xC6, 0x00, 0xA0, 0x00, 0x40])

	#step 6: press lift button to target floor
	press_lift_button(lift_addr, 0x82, [0xC4, 0x00, 0xA0, 0x01, 0x00], "/bluetooth_mesh/receive", [0xC6, 0x00, 0xA0, 0x01, 0x00])
	

	#change map and position
	rospack = rospkg.RosPack()
	map_name = map_name_dict[target_floor]
	map_path = rospack.get_path("magni_lidar") + "/maps/" + map_name + ".yaml"
	change_map_thread = threading.Thread(target = os.system, args=["rosrun map_server map_server " + map_path])
	change_map_thread.start()

	command = change_pose_command(target_floor_nav_point[0], target_floor_nav_point[1], target_floor_nav_point[2])

	os.system(command)

	print("changed pose and map")

	curr_x = target_floor_nav_point[0]
	curr_y = target_floor_nav_point[1]

	#step 7: keep waiting for the lift to arrive target floor and open the door
	subscribe_till_response(lift_addr, 0x82, [0xc1, 0x00, 0xa0, 0x01, 0x01], "/bluetooth_mesh/receive", 0.5, [0xc2, 0x00, 0xa0, 0x01, 0x01])
	print("arrived")
	curr_floor = target_floor

	#add open door
	subscribe_till_response(target_lobby_addr, 0x82, [0xC4, 0x00, 0xA0, 0x00, 0x80], "/bluetooth_mesh/receive", 2, [0xC6, 0x00, 0xA0, 0x00, 0x80])

#function to check a point inside the door or outside
def is_area(list_of_points, x, y):
	for item in list_of_points:
		x_0 = item[0]
		x_1 = item [1]
		y_0 = item[2]
		y_1 = item[3]
		if (x >= x_0 and x <= x_1) and (y >= y_0 and y <= y_1):
			return True
	return False


#go to a point in the same floor intelligently
def same_floor_nav(goal_x, goal_y, goal_theta):
	global curr_floor, curr_x, curr_y
	outside_door_coord = {0: [0, 0, 0], 3: [3.5, -0.56, -1.57]}
	inside_door_coord = {0: [0, 0, 0], 3: [3.3, -7, 1.57]}
	door_addr = {0: 0x800, 1: 0x810, 2: 0x820, 3: 0x830}
	#check the start point and end point of the navigation is inside or outside, inside = 0, outside = 1

	# the lists are presented in [x0, x1, y0, y1]
	inside_area = {3: [[-18, 4.3, -17.2, -5.5]]}
	outside_area = {3: [[-0.9, 4.7, -0.9, 2.2]]}

	if is_area(inside_area[curr_floor], curr_x, curr_y):
		start_point = 0

	elif is_area(outside_area[curr_floor], curr_x, curr_y):
		start_point = 1

	else:
		print("error! wrong current position")
		return


	if is_area(inside_area[curr_floor], goal_x, goal_y):
		end_point = 0

	elif is_area(outside_area[curr_floor], goal_x, goal_y):
		end_point = 1

	else:
		print(goal_x, goal_y)
		print("error! invalid goal position")
		return


	if (start_point == end_point):
		print("directly go to goal")
		go_to(goal_x, goal_y, goal_theta)

	elif (start_point == 0):
		print("go inside door, open door, then go to goal")
		first_goal = inside_door_coord[curr_floor]
		door_address = door_addr[curr_floor]
		go_to(first_goal[0], first_goal[1], first_goal[2])
		press_lift_button(door_address, 0x82, [0xC4, 0x00, 0xA0, 0x00, 0x80], "/bluetooth_mesh/receive", [0xC6, 0x00, 0xA0, 0x00, 0x80])
		print("opened")
		rospy.sleep(2.5)
		go_to(goal_x, goal_y, goal_theta)

	else:
		print("go outside door, open door, then go to goal")
		first_goal = outside_door_coord[curr_floor]
		door_address = door_addr[curr_floor]
		go_to(first_goal[0], first_goal[1], first_goal[2])
		press_lift_button(door_address, 0x82, [0xC4, 0x00, 0xA0, 0x00, 0x80], "/bluetooth_mesh/receive", [0xC6, 0x00, 0xA0, 0x00, 0x80])
		print("opened")
		rospy.sleep(2.5)
		go_to(goal_x, goal_y, goal_theta)

#multi floor navigation
def multi_floor_nav (target_floor, target_x, target_y, target_theta):
	global curr_floor, curr_x, curr_y
	print("curr x: ", curr_x, "curr y: ", curr_y)
	if (curr_floor != target_floor):
		change_floor(target_floor)
	same_floor_nav(target_x, target_y, target_theta)


if __name__ == "__main__":
	rospy.init_node("testing", anonymous = True)
	
	pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size =10)

	amcl = rospy.wait_for_message('amcl_pose', PoseWithCovarianceStamped)
	curr_x = amcl.pose.pose.position.x
	curr_y = amcl.pose.pose.position.y


	move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
	multi_floor_nav(3, 0, 0, 0)
	multi_floor_nav(3, 3.3, -7, 0)
	
	
