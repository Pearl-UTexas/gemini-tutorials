#!/usr/bin/env python 

import rospy
import rospkg
import copy, pickle
import std_msgs
import vector_msgs
import geometry_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from hlpr_manipulation_utils.arm_moveit2 import *


joint_traj = RobotTrajectory()

def traj_callback(msg):
	# Extract trajectory from msg
	global joint_traj
	joint_traj = copy.deepcopy(msg.trajectory[0])

def clean_joint_traj(msg):
	msg = copy.deepcopy(msg)
	for pt in msg.points:
		pt.velocities = []
		pt.accelerations = []
		pt.effort = []

	return msg

def tweak_time(msg, time_scale):
	msg = copy.deepcopy(msg)
	for pt in msg.points:
		pt.time_from_start /= 1.*time_scale
	return msg

def main():
	global joint_traj
	arm = ArmMoveIt(planning_frame='linear_actuator_link', _arm_name='right')

	start = [0.51, 4.87, 2.21, 1.75, 2.64, 1.61, 0.52]
	end = [1.689, 4.739, 0.118, 4.313, -0.148, 1.150, 3.340]

	# Home Pose
	jointTarget = start
	arm.move_to_joint_pose(jointTarget)
	rospy.sleep(3)


	rospy.Subscriber("/move_group/display_planned_path", DisplayTrajectory, traj_callback)

	# Generate a trajectory (Use demo1)
	raw_input('Executing Arm trajectory command. Press Enter!')
	jointTarget = end
	arm.move_to_joint_pose(jointTarget)
	rospy.sleep(5)

	# Save joint trajectory
	joint_traj_saved = clean_joint_traj(joint_traj.joint_trajectory)

	# Going Back for Playback
	jointTarget = start
	for i in range(3):
		arm.move_to_joint_pose(jointTarget)
		rospy.sleep(1)
		print 'Iterating over start point'

	rospy.sleep(3)

	current_joint_pt = JointTrajectoryPoint()
	current_joint_pt.positions = arm.get_current_pose()
	# joint_traj_saved.points = [current_joint_pt] + joint_traj_saved.points
	joint_traj_saved.points[0] = current_joint_pt
	# for i in range(1,len(joint_traj_saved.points)):
	# 	joint_traj_saved.points[i].time_from_start.nsecs += 500000000


	# Do FK for every point in traj to convert to eef trajectory
	ee_traj = []

	for i, pt in enumerate(joint_traj_saved.points):
		ee_pose = arm.get_FK(root='linear_actuator_link', joints=pt.positions)[0]
		ee_pose.header.seq = i
		ee_pose.header.stamp = pt.time_from_start.to_sec()
		ee_traj.append(copy.deepcopy(ee_pose))

	# print 'ee_traj =', ee_traj

	# # save it
	# rospack = rospkg.RosPack()
	# cwd = rospack.get_path('gemini_tutorials')
	# filename = cwd + '/src/ee_trajectory.pickle'
	# file = open(filename,'wb')
	# pickle.dump(ee_traj, file) 
	# file.close()

	# Playback tracjectory
	raw_input('Playing back the trajectory, Press Enter!')
	#for ee_pose in ee_traj:
	#	arm.move_to_ee_pose(ee_pose.pose)

	# print 'Going to the first ee pose in ee traj'
	# joint_traj = arm._ee_traj_to_joint_traj(ee_traj)
	# arm.move_to_joint_pose(joint_traj.points[0].positions)
	# print 'Exercuting the rest traj'
	# arm.execute_joint_trajectory(joint_traj)

	arm.execute_ee_trajectory(ee_traj, use_custom_time=False)
	#joint_traj_saved = tweak_time(joint_traj_saved, 2)
	#arm.execute_joint_trajectory(joint_traj_saved, use_custom_time=True)

	# Shutdown
	print "shutting down!"
	# shut down moveit
	moveit_commander.roscpp_shutdown()
	moveit_commander.os._exit(0)




if __name__=='__main__':
	rospy.init_node('demo_manipulation', anonymous=True)
	main()
	# rospy.spin()
	
	
	
	
	
