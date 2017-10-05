#!/usr/bin/env python 

import rospy, copy
import std_msgs
import vector_msgs
import geometry_msgs.msg
from ar_track_alvar_msgs.msg import *
from hlpr_manipulation_utils.arm_moveit2 import *
from hlpr_manipulation_utils.manipulator import Gripper

goal_pose = geometry_msgs.msg.PoseStamped()

def arPoseMarkerCallback(msg):
	global goal_pose

	if(len(msg.markers)>0):
		# marker Pose
		marker_pose_stamped = msg.markers[0].pose
		goal_pose = copy.deepcopy(marker_pose_stamped)
		goal_pose.pose.position.x -= 0.20
		goal_pose.pose.position.y += 0.20
	else:
		# Only for sim
		goal_pose.pose.position.x = 0.798
		goal_pose.pose.position.y = -0.115
		goal_pose.pose.position.z = 0.205
		goal_pose.pose.orientation.x = -0.025
		goal_pose.pose.orientation.y = 0.730
		goal_pose.pose.orientation.z = 0.064
		goal_pose.pose.orientation.w = 0.680

def add_collision_objects(arm):
	### Table
	table_pose = geometry_msgs.msg.PoseStamped()
	table_pose.header.frame_id = "base_link"
	# table_pose.pose.position.x = 0.6+0.8;
	table_pose.pose.position.x = 1.25+.3; #distance + half width CHANGE THIS TO MATCH HOW FAR TABLE IS
	table_pose.pose.position.y = 0;
	table_pose.pose.position.z = .4; # half of height
	table_pose.pose.orientation.x = 0;
	table_pose.pose.orientation.y = 0;
	table_pose.pose.orientation.z = 0;
	table_pose.pose.orientation.w = 1;

	table_scale = [0.6,1.23,.8] # check if height is too low

	arm.scene.add_box("table",table_pose,table_scale)
	rospy.sleep(2)

	### Floor
	floor_pose =  geometry_msgs.msg.PoseStamped()
	floor_pose.header.frame_id = "base_link"
	floor_pose.pose.position.x = 0
	floor_pose.pose.position.y = 0
	floor_pose.pose.position.z = 0.1
	floor_pose.pose.orientation.x = 0;
	floor_pose.pose.orientation.y = 0;
	floor_pose.pose.orientation.z = 0;
	floor_pose.pose.orientation.w = 1;
	floor_scale = [5,5,0.01]

	arm.scene.add_box("floor",floor_pose,floor_scale)
	rospy.sleep(2)
	return

def attach_collision_object(arm):
	leg_pose = geometry_msgs.msg.PoseStamped()
	leg_pose.header.frame_id = "right_ee_link"
	leg_pose.pose.position.x = 0 ## Need to Set these
	leg_pose.pose.position.y = 0
	leg_pose.pose.position.z = 0.15
	leg_pose.pose.orientation.x = 0;
	leg_pose.pose.orientation.y = 0;
	leg_pose.pose.orientation.z = 0;
	leg_pose.pose.orientation.w = 1;

	leg_scale = [0.05,0.05,0.3]
	arm.scene.attach_box("right_ee_link","leg",leg_pose,leg_scale)
	return


def main():
	tilt_publisher = rospy.Publisher('/tilt_controller/command', std_msgs.msg.Float64, queue_size=10)
	arm = ArmMoveIt(planning_frame='linear_actuator_link', _arm_name='right')
	gripper = Gripper()
	rospy.Subscriber('/ar_pose_marker', AlvarMarkers, arPoseMarkerCallback)
	rospy.sleep(1)

	# Setup collison Objects
	raw_input('Adding collision objects. Press Enter!')
	add_collision_objects(arm)

	# Attach collision object as the leg
	attach_collision_object(arm)

	# Set goal point on the table using ar_tag
	global goal_pose

	# return to home pose
	raw_input('Returning to home pose. Press Enter!')
	jointTarget = [1.689, 4.739, 0.118, 4.313, -0.148, 1.150, 3.340]
	arm.move_to_joint_pose(jointTarget)
	rospy.sleep(3)

	# go to goal point
	raw_input('Executing Arm trajectory command. Press Enter!')
	tarPose = copy.deepcopy(goal_pose.pose)
	tarPose.position.z += 0.10
	arm.move_to_ee_pose(tarPose)
	rospy.sleep(3)

	# drop the object
	tarPose = copy.deepcopy(goal_pose.pose)
	tarPose.position.z += 0.04
	arm.move_to_ee_pose(tarPose)
	rospy.sleep(1)

	# Open Gripper
	gripper.open(100)
	rospy.sleep(3)

	# return to home pose
	jointTarget = [1.689, 4.739, 0.118, 4.313, -0.148, 1.150, 3.340]
	arm.move_to_joint_pose(jointTarget)
	rospy.sleep(3)

	# Shutdown
	print "shutting down!"
	# shut down moveit
	moveit_commander.roscpp_shutdown()
	moveit_commander.os._exit(0)


if __name__=='__main__':
	rospy.init_node('demo_manipulation', anonymous=True)
	main()
	# rospy.spin()
	
	
	
	
	
