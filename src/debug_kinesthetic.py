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
	rospy.sleep(7)

	# Save joint trajectory
	joint_traj_saved = copy.deepcopy(joint_traj.joint_trajectory)

	# Going Back for Playback
	jointTarget = start
	arm.move_to_joint_pose(jointTarget)
	rospy.sleep(5)

	# Do FK for every point in traj to convert to eef trajectory
	current_joint_pt = JointTrajectoryPoint()
	current_joint_pt.positions = arm.get_current_pose()
	joint_traj_saved.points = [current_joint_pt] + joint_traj_saved.points
	for i in range(1,len(joint_traj_saved.points)):
		joint_traj_saved.points[i].time_from_start.nsecs += 500000000

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
	#arm.execute_ee_trajectory(ee_traj)
	arm.execute_joint_trajectory(joint_traj_saved)

	# Shutdown
	print "shutting down!"
	# shut down moveit
	moveit_commander.roscpp_shutdown()
	moveit_commander.os._exit(0)




if __name__=='__main__':
	rospy.init_node('demo_manipulation', anonymous=True)
	main()
	# rospy.spin()
	
	
	
	
	
