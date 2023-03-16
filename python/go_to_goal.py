import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

def go_to_goal(point, orientation):
	rospy.init_node("navigation_goal", anonymous = True)
	client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
	goal = MoveBaseGoal()
	q = quaternion_from_euler(0, 0, orientation)
	goal.target_pose.header.frame_id = "map"
	goal.target_pose.pose.orientation.x = q[0]
	goal.target_pose.pose.orientation.y = q[1]
	goal.target_pose.pose.orientation.z = q[2]
	goal.target_pose.pose.orientation.w = q[3]
	goal.target_pose.pose.position.x = point[0]
	goal.target_pose.pose.position.y = point[1]

	print(goal)
	client.wait_for_server()
	client.send_goal_and_wait(goal)

go_to_goal((1, 0), 1.57)