#!/usr/bin/env python 

import rospy
import rospkg
import copy, pickle
import std_msgs
import vector_msgs
import geometry_msgs.msg
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

	# Home Pose
	jointTarget = [0.51, 4.87, 2.21, 1.75, 2.64, 1.61, 0.52]
	arm.move_to_joint_pose(jointTarget)
	rospy.sleep(3)


	rospy.Subscriber("/move_group/display_planned_path", DisplayTrajectory, traj_callback)

	# Generate a trajectory (Use demo1)
	raw_input('Executing Arm trajectory command. Press Enter!')
	jointTarget = [1.689, 4.739, 0.118, 4.313, -0.148, 1.150, 3.340]
	arm.move_to_joint_pose(jointTarget)
	rospy.sleep(7)

	# Read Global and Do FK for every point in traj to convert to eef trajectory
	ee_traj = []
	i = 0

	for pt in joint_traj.joint_trajectory.points:
		ee_pose = arm.get_FK(root='linear_actuator_link', joints=pt.positions)[0]
		ee_pose.header.seq = i
		ee_pose.header.stamp = pt.time_from_start.to_sec()
		ee_traj.append(copy.deepcopy(ee_pose))
		i += 1

	# print 'ee_traj =', ee_traj

	# # save it
	# rospack = rospkg.RosPack()
	# cwd = rospack.get_path('gemini_tutorials')
	# filename = cwd + '/src/ee_trajectory.pickle'
	# file = open(filename,'wb')
	# pickle.dump(ee_traj, file) 
	# file.close()

	# Going Back for Playback
	jointTarget = [0.51, 4.87, 2.21, 1.75, 2.64, 1.61, 0.52]
	arm.move_to_joint_pose(jointTarget)
	rospy.sleep(5)

	# Playback tracjectory
	raw_input('Playing back the trajectory, Press Enter!')
	arm.execute_ee_trajectory(ee_traj)

	# Shutdown
	print "shutting down!"
	# shut down moveit
	moveit_commander.roscpp_shutdown()
	moveit_commander.os._exit(0)




if __name__=='__main__':
	rospy.init_node('demo_manipulation', anonymous=True)
	main()
	# rospy.spin()
	
	
	
	
	
