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

#publisher function
def publish_bluetooth(dest, source, data):
	global kill_navigation, pub_bluetooth
	if kill_navigation:
		return 0
	the_message = Message()
	the_message.msg.dest = dest
	the_message.msg.source = source
	the_message.msg.data = data
	#pub_bluetooth = rospy.Publisher('/bluetooth_mesh/send', Message, queue_size = 10)
	#rospy.sleep(0.1)
	pub_bluetooth.publish(the_message)
	#print("pub", dest, data)

		
		
#subscriber function (subscribe continuously until there is a response)
def subscribe_till_response(bluetooth_dest, bluetooth_source, bluetooth_data, topic, seconds_to_retry, expected_message):
	global kill_navigation
	while True:
		if kill_navigation:
			return 0
		try:
			publish_bluetooth(bluetooth_dest, bluetooth_source, bluetooth_data)
			received = rospy.wait_for_message(topic, Message, timeout=seconds_to_retry)
			print('received data =', received, 'expected = ', expected_message)
			if (compare_list(received.msg.data, expected_message) == True):
				return

		except:
			print("not yet")

#function for press lift
def wait_lift(bluetooth_dest, bluetooth_source, bluetooth_data, topic, seconds_to_retry, expected_message):
	global lift_addr, kill_navigation
	while lift_addr is None:
		try:
			if kill_navigation:
				return 0
			publish_bluetooth(bluetooth_dest, bluetooth_source, bluetooth_data)
			received = rospy.wait_for_message(topic, Message, timeout=seconds_to_retry)
			print('received data =', received, 'expected = ', expected_message, "lift addr", lift_addr)
			if (compare_list(received.msg.data, expected_message) == True):
				lift_addr = received.msg.source
				print("====================================end===============================")
				return
			rospy.sleep(0.5)

		except Exception as e:
			pass
	lift_addr = None
	print("wait lift ended")



def wait_two_lift(left_lobby, right_lobby):
	global lift_addr
	while lift_addr == None:
			print("lift addr: ", lift_addr)
			publish_bluetooth(left_lobby, 0x82, [0xc1, 0x00, 0xa0, 0x01, 0x01])
			rospy.sleep(0.4)
			publish_bluetooth(right_lobby, 0x82, [0xc1, 0x00, 0xa0, 0x01, 0x01])
			rospy.sleep(0.4)
	lift_addr = None
			
			
def check_two_lift():
	global lift_addr
	while lift_addr == None:
		try:
			received = rospy.wait_for_message( "/bluetooth_mesh/receive", Message, timeout=0.5)
			print(received)
			if (compare_list(received.msg.data, [0xC2, 0x00, 0xA0, 0x01, 0x01]) == True):
				lift_addr = received.msg.source
				print("====================================end 1===============================")
		except Exception as e:
			print(e)
	

def subscribe_for_second(topic, expected_message, second):
	try:
		received = rospy.wait_for_message(topic, Message, timeout=second)
		print("received: ", received)
		if (compare_list(received.msg.data, expected_message) == True):
			return True
		else:
			return False

	except:
		return False
		
def subscribe_lift_for_second(topic, expected_message, second):
	try:
		received = rospy.wait_for_message(topic, Message, timeout=second)
		print("received: ", received)
		if (compare_list(received.msg.data, expected_message) == True):
			return received.msg.source
		else:
			return None

	except:
		return None

def press_lift_button(dest, source, data, sub_topic, expected_message):
	button_pressed = False
	while not button_pressed:
		global kill_navigation
		if kill_navigation:
			return 0
		print("life button not pressed yet!")
		publish_bluetooth(dest, source, data)
		button_pressed = subscribe_for_second(sub_topic, expected_message, 2)
	print("lift button pressed")

def check_lift_once(dest, source, data, topic, second, expected_message):
	publish_bluetooth(dest, source, data)
	result = subscribe_lift_for_second(topic, expected_message, second)
	return result 
	

def checking(left_addr, right_addr):
	while True:

		print("sub left lift")
		left_result = check_lift_once(left_addr, 0x82, [0xc1, 0x00, 0xa0, 0x01, 0x01], "/bluetooth_mesh/receive", 1, [0xC2, 0x00, 0xA0, 0x01, 0x01])
		if left_result !=None:
			
			return left_result
	
		print("===========================sub right lift")
		right_result = check_lift_once(right_addr, 0x82, [0xc1, 0x00, 0xa0, 0x01, 0x01], "/bluetooth_mesh/receive", 1, [0xC2, 0x00, 0xA0, 0x01, 0x01])
		if right_result !=None:
			
			return right_result


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
	move_base_ended = False
	def status_monitor():
		global kill_navigation
		while not move_base_ended:
			if kill_navigation:
				move_base_client.cancel_goal()
				return 0
	status_thread = threading.Thread(target = status_monitor)
	status =status_thread.start()
	wait = move_base_client.wait_for_result()
	move_base_ended = True
	status_thread.join()	
	if status ==0:
		return 0

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
      y: ''' + str(y) +''' 
      z: 0.0
    orientation: 
      x: ''' + str(q[0]) +'''
      y: ''' + str(q[1]) +'''
      z: ''' + str(q[2]) +'''
      w: ''' + str(q[3]) + ''' 
  covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]"'''

	return command

# function to go from a floor to another floor
curr_floor = 4
curr_x = 0
curr_y = 0
lift_addr = None
def change_floor(target_floor):
	global curr_floor, curr_x, curr_y, robot_state, lift_addr
	#add target floor command in hex 

	map_name_dict = {0: "gf_2lift_2", 3: "3f_2lift", 4: "4f_map"}

	left_lobby_addr_dict = {0: 0x400, 1: 0x410, 2: 0x420, 3: 0x430, 4:0x440}
	right_lobby_addr_dict = {0: 0x500, 1: 0x510, 2: 0x520, 3: 0x530, 4:0x540}
	left_lobby_list = [0x400, 0x410, 0x420, 0x430, 0x440]
	right_lobby_list = [0x500, 0x510, 0x520, 0x530, 0x540]
	lift_addr_dict = {"left": 0x330, "right": 0x320}

	left_lobby_addr = left_lobby_addr_dict[curr_floor]
	right_lobby_addr = right_lobby_addr_dict[curr_floor]
	print(target_floor, curr_floor)
	#target_lobby_addr = lobby_addr_dict[target_floor]

	curr_floor_lift_location = {0 : [0, 0, 0], 3 : [0.45, -0.5, -1.57], 4: [14, 24.5, 3.14]}
	inside_lift_location_left = {0 : [-1.35, -11, 1.57], 3 : [0.45, -3, 1.57], 4: [13.8, 21.2, 1.57]}
	inside_lift_location_right = {0 : [-1, -4, -1.57], 3 : [0.5, 4, -1.57], 4: [13.8, 27.9, -1.57]}
	
	curr_floor_nav_point = curr_floor_lift_location[curr_floor]
	#curr_floor_lift_nav_point = curr_floor_inside_lift_location[curr_floor]
	#target_floor_nav_point = target_floor_inside_lift_location[target_floor]
	same_floor_nav(curr_floor_nav_point[0], curr_floor_nav_point[1], curr_floor_nav_point[2])
	curr_x = curr_floor_nav_point[0]
	curr_y = curr_floor_nav_point[1]


	robot_state = "waiting"
	#send bluetooth message to press lift button, wait for response
	#send bluetooth message to open door
	status = press_lift_button(left_lobby_addr, 0x82, [0xC4, 0x00, 0xA0, 0x02, 0x00], "/bluetooth_mesh/receive", [0xC6, 0x00, 0xA0, 0x02, 0x00])
	if status == 0:
		print("status 1 ended")
		return 0


	#keep checking if lift is arrived and lift door opened
	#status = subscribe_till_response(lobby_addr, 0x82, [0xc1, 0x00, 0xa0, 0x01, 0x01], "/bluetooth_mesh/receive", 0.5, [0xC2, 0x00, 0xA0, 0x01, 0x01])
	
	#right_lift_thread = threading.Thread(target = wait_lift, args = [right_lobby_addr, 0x82, [0xc1, 0x00, 0xa0, 0x01, 0x01], "/bluetooth_mesh/receive", 0.5, [0xC2, 0x00, 0xA0, 0x01, 0x01]])
	#right_lift_thread.start()
	
	#wait_lift(left_lobby_addr, 0x82, [0xc1, 0x00, 0xa0, 0x01, 0x01], "/bluetooth_mesh/receive", 0.5, [0xC2, 0x00, 0xA0, 0x01, 0x01])

	lift_addr  = checking(left_lobby_addr, right_lobby_addr)
	if lift_addr in left_lobby_list:
		curr_lift = "left"
	elif lift_addr in right_lobby_list:
		curr_lift = "right"
	else:
		print("i dont know what is ", lift_addr)
		return 
	print("========================lift arrived ", curr_lift)
	
	
	if curr_lift == "left":
		print("inside left")
		#step 4: hold the life, then go into the lift
		#subscribe_till_response(left_lobby_addr, 0x82, [0xC4, 0x00, 0xA0, 0x00, 0x80], "/bluetooth_mesh/receive", 5, [0xC6, 0x00, 0xA0, 0x00, 0x80])
		robot_state = "running"
		curr_floor_lift_nav_point = inside_lift_location_left[curr_floor]
		go_to(curr_floor_lift_nav_point[0], curr_floor_lift_nav_point[1], curr_floor_lift_nav_point[2])
		curr_x = curr_floor_lift_nav_point[0]
		curr_y = curr_floor_lift_nav_point[1]
		print("entered lift")
		robot_state = "waiting"
		#step 5: release the open lift door command
		subscribe_till_response(left_lobby_addr, 0x82, [0xC4, 0x00, 0xA0, 0x00, 0x40], "/bluetooth_mesh/receive", 2, [0xC6, 0x00, 0xA0, 0x00, 0x40])
		#step 6: press lift button to target floor
		
		press_lift_button(lift_addr_dict["left"], 0x82, [0xC4, 0x00, 0xA0, 0x01, 0x00], "/bluetooth_mesh/receive", [0xC6, 0x00, 0xA0, 0x01, 0x00])
		#change map and position
		rospack = rospkg.RosPack()
		map_name = map_name_dict[target_floor]
		map_path = rospack.get_path("magni_lidar") + "/maps/" + map_name + ".yaml"
		change_map_thread = threading.Thread(target = os.system, args=["rosrun map_server map_server " + map_path])
		change_map_thread.start()
		target_floor_nav_point = inside_lift_location_left[target_floor]
		command = change_pose_command(target_floor_nav_point[0], target_floor_nav_point[1], target_floor_nav_point[2])

		os.system(command)

		print("changed pose and map")

		curr_x = target_floor_nav_point[0]
		curr_y = target_floor_nav_point[1]
		#step 7: keep waiting for the lift to arrive target floor and open the door
		subscribe_till_response(lift_addr_dict["left"], 0x82, [0xc1, 0x00, 0xa0, 0x01, 0x01], "/bluetooth_mesh/receive", 0.5, [0xc2, 0x00, 0xa0, 0x01, 0x01])
		#add open door
		subscribe_till_response(left_lobby_addr_dict[target_floor], 0x82, [0xC4, 0x00, 0xA0, 0x00, 0x80], "/bluetooth_mesh/receive", 2, [0xC6, 0x00, 0xA0, 0x00, 0x80])
		
	
	elif curr_lift == "right":
		print("inside right")
		#step 4: hold the life, then go into the lift
		#subscribe_till_response(right_lobby_addr, 0x82, [0xC4, 0x00, 0xA0, 0x00, 0x80], "/bluetooth_mesh/receive", 5, [0xC6, 0x00, 0xA0, 0x00, 0x80])
		robot_state = "running"
		curr_floor_lift_nav_point = inside_lift_location_right[curr_floor]
		go_to(curr_floor_lift_nav_point[0], curr_floor_lift_nav_point[1], curr_floor_lift_nav_point[2])
		curr_x = curr_floor_lift_nav_point[0]
		curr_y = curr_floor_lift_nav_point[1]
		print("entered lift")
		robot_state = "waiting"
		#step 5: release the open lift door command
		subscribe_till_response(right_lobby_addr, 0x82, [0xC4, 0x00, 0xA0, 0x00, 0x40], "/bluetooth_mesh/receive", 2, [0xC6, 0x00, 0xA0, 0x00, 0x40])
		#step 6: press lift button to target floor
		
		press_lift_button(lift_addr_dict["right"], 0x82, [0xC4, 0x00, 0xA0, 0x01, 0x00], "/bluetooth_mesh/receive", [0xC6, 0x00, 0xA0, 0x01, 0x00])
		#change map and position
		rospack = rospkg.RosPack()
		map_name = map_name_dict[target_floor]
		map_path = rospack.get_path("magni_lidar") + "/maps/" + map_name + ".yaml"
		change_map_thread = threading.Thread(target = os.system, args=["rosrun map_server map_server " + map_path])
		change_map_thread.start()
		target_floor_nav_point = inside_lift_location_right[target_floor]
		command = change_pose_command(target_floor_nav_point[0], target_floor_nav_point[1], target_floor_nav_point[2])

		os.system(command)

		print("changed pose and map")

		curr_x = target_floor_nav_point[0]
		curr_y = target_floor_nav_point[1]
		#step 7: keep waiting for the lift to arrive target floor and open the door
		subscribe_till_response(lift_addr_dict["right"], 0x82, [0xc1, 0x00, 0xa0, 0x01, 0x01], "/bluetooth_mesh/receive", 0.5, [0xc2, 0x00, 0xa0, 0x01, 0x01])
		#add open door
		subscribe_till_response(right_lobby_addr_dict[target_floor], 0x82, [0xC4, 0x00, 0xA0, 0x00, 0x80], "/bluetooth_mesh/receive", 2, [0xC6, 0x00, 0xA0, 0x00, 0x80])
		
	else:
		print("error, unknown lift")
		

	print("arrived target floor")
	curr_floor = target_floor

	

	

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
	global curr_floor, curr_x, curr_y, robot_state
	robot_state = "running"
	outside_door_coord = {0: [2, -4.8, 1.57], 3: [3.5, -0.56, -1.57], 4:[17, 22, -1.57]}
	inside_door_coord = {0: [0, 0, 0], 3: [3.3, -7, 1.57], 4:[16.5, 16.5, 1.57]}
	door_addr = {0: 0x800, 1: 0x810, 2: 0x820, 3: 0x830, 4: 0x842}
	#check the start point and end point of the navigation is inside or outside, inside = 0, outside = 1

	# the lists are presented in [x0, x1, y0, y1]
	inside_area = {0:[[14.3, 7.1, -1.5, 6.9]], 3: [[-18, 4.3, -17.2, -5.5]], 4:[[-11, 17, -31, 20]]}
	outside_area = {0:[[-14.3, 7.1, -12.5, -5.3]], 3: [[-0.9, 4.7, -0.9, 2.2]], 4:[[13, 18, 23, 30]]}

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
		print("error! invalid goal position")
		return


	if (start_point == end_point):
		print("directly go to goal")
		status = go_to(goal_x, goal_y, goal_theta)
		if status ==0:
			print("same floor nav end 1")
			return 0
	elif (start_point == 0):
		print("go inside door, open door, then go to goal")
		first_goal = inside_door_coord[curr_floor]
		door_address = door_addr[curr_floor]
		status = go_to(first_goal[0], first_goal[1], first_goal[2])
		if status == 0:
			print("same floor nav end 2")
			return 0
		robot_state = "waiting"
		status = press_lift_button(door_address, 0x82, [0xC4, 0x00, 0xA0, 0x00, 0x80], "/bluetooth_mesh/receive", [0xC6, 0x00, 0xA0, 0x00, 0x80])
		if status == 0 :
			print("same floor nav end 2")
			return 0
		print("opened")
		rospy.sleep(3.5)
		robot_state = "running"
		status = go_to(goal_x, goal_y, goal_theta)
		if status == 0 :
			print("same floor nav end 2")
			return 0


	else:
		print("go outside door, open door, then go to goal")
		print("curr floor: ", curr_floor)
		first_goal = outside_door_coord[curr_floor]
		door_address = door_addr[curr_floor]
		status = go_to(first_goal[0], first_goal[1], first_goal[2])
		robot_state = "waiting"
		if status == 0 :
			print("same floor nav end 3")
			return 0
		status = press_lift_button(door_address, 0x82, [0xC4, 0x00, 0xA0, 0x00, 0x80], "/bluetooth_mesh/receive", [0xC6, 0x00, 0xA0, 0x00, 0x80])
		if status == 0 :
			print("same floor nav end 3")
			return 0

		print("opened")
		rospy.sleep(2.5)
		robot_state = "running"
		status = go_to(goal_x, goal_y, goal_theta)
		if status == 0 :
			print("same floor nav end 3")
			return 0


#multi floor navigation
def multi_floor_nav (target_floor, target_x, target_y, target_theta):
	global curr_floor, curr_x, curr_y, kill_navigation, robot_state
	print("curr x: ", curr_x, "curr y: ", curr_y)
	if (curr_floor != target_floor):
		print("change floor first, then go to goal")
		status = change_floor(target_floor)
		if status ==0:
			print("multi floor nav 1")
			return 0
	else:
		print("go to goal in same floor")
	status = same_floor_nav(target_x, target_y, target_theta)
	if status == 0:
		print("multi floor nav 1")
		kill_navigation = False
		return 0
	robot_state = "idle"

robot_state = "idle"

#call back function for amcl
def amcl_callback(amcl):
	global curr_x, curr_y, robot_state
	curr_x = amcl.pose.pose.position.x
	curr_y = amcl.pose.pose.position.y
	messge = {"agv_id": "11", "agv_name": "EIABot", "state": robot_state, "type": "coord", "location": "", "floor": curr_floor, "coord":{"x": curr_x, "y": curr_y}}
	#client.publish("/ROBOT/STATE", str(messge))

kill_navigation  = False

def update_amcl():
	r = rospy.Rate(1)
	while True:
		amcl_sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, amcl_callback)
		r.sleep()
		
		
goal_point = {0:{},
		1:{},
		2:{},
		3:{"1": [-4, -8, 0], "2": [0.45, -0.5, -1.57]}}
def on_message(client, userdata, msg):
	global kill_navigation, robot_state, goal_point
	result = msg.payload.decode("utf-8","ignore")
	result_json = json.loads(result)	
	
	agv_id = result_json["agv_id"]
	goal_type = result_json["type"]
	floor = result_json["floor"]

	

	#if result == "stop":
	#	kill_navigation = True


	if agv_id =="11" and goal_type == "coord":
		robot_state = "running"
		x = result_json["coord"]["x"]
		y = result_json["coord"]["y"]
		nav_thread = threading.Thread(target = multi_floor_nav, args = [floor, x, y, 1.57])
		status = nav_thread.start()

	if agv_id == "11" and goal_type == "name":
		point = result_json["location"]
		goal_point_dict = goal_point[floor]
		x = goal_point_dict[point][0]
		y = goal_point_dict[point][1]
		robot_state = "running"
		nav_thread = threading.Thread(target = multi_floor_nav, args = [floor, x, y, 1.57])
		status = nav_thread.start()

if __name__ == "__main__":
	rospy.init_node("testing", anonymous = True)
	pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size =10)
	pub_bluetooth = rospy.Publisher('/bluetooth_mesh/send', Message, queue_size = 100)
	port = 1883

	#client = paho.Client()
	#client.connect("ia.ic.polyu.edu.hk", port, 60)
	#client.subscribe("/ROBOT/GOTO")
	#client.on_message = on_message
	#client.loop_start()



	amcl_thread = threading.Thread(target = update_amcl)
	amcl_thread.start()

	move_base_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
	#multi_floor_nav(4, 16, 15, 1.57)
	#multi_floor_nav(4, 15.5, 24.5, 3.14)
	multi_floor_nav(3, 0.5, 4, -1.57)
	
