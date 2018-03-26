#!/usr/bin/env python 

import rospy
import rospkg
import copy, pickle
import std_msgs
import vector_msgs
import geometry_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from control_msgs.msg import FollowJointTrajectoryGoal
from hlpr_manipulation_utils.arm_moveit2 import *
from tf.transformations import *
import numpy as np
from cartesian_trajectory_planner.trajopt_planner import *


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

def pose_array_to_traj(poses, times, frame_id='linear_actuator_link'):
	traj = JointTrajectory()
	traj.header.frame_id = frame_id
	traj.joint_names = ['right_eef_x', 'right_eef_y', 'right_eef_z', 'right_eef_roll', 'right_eef_pitch', 'right_eef_yaw']

	traj.points = []
	for i, pose in enumerate(poses):
		pt = JointTrajectoryPoint()
		rpy = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
		pt.positions = [pose.postion.x, pose.position.y, pose.position.z] + rpy
		pt.time_from_start = times[i]
		traj.points.append(pt)

	return traj

def joint_array_to_traj(joint_array):
	traj = JointTrajectory()
	# traj.header.frame_id = frame_id
	traj.joint_names = ['right_joint_1', 'right_joint_2', 'right_joint_3', 'right_joint_4', 'right_joint_5', 'right_joint_6', 'right_joint_7']
	traj.points = []
	for i, joint in enumerate(joint_array):
		pt = JointTrajectoryPoint()
		pt.positions = joint
		traj.points.append(pt)

	return traj

def simplify_plan(plan, arm):
	plan = copy.deepcopy(plan)

	joints_simplified = []
	for pt in plan.joint_trajectory.points:
		joints_simplified.append(arm._simplify_joints(list(pt.positions)))
		# joints_simplified.append(list(pt.positions))

	print 'plan =', plan.joint_trajectory.points
	plan.joint_trajectory.points = []
	pt = JointTrajectoryPoint()
	pt.positions = joints_simplified[0]
	plan.joint_trajectory.points.append(copy.deepcopy(pt))

	for i in range(1,len(joints_simplified)):
		pt.positions = []

		for j in range(len(joints_simplified[i])):
			pt.positions.append(arm._nearest_equivalent_angle(joints_simplified[i][j], joints_simplified[i-1][j]))
		
		plan.joint_trajectory.points.append(copy.deepcopy(pt))

	return plan

## Debugging
def poseStamped2Array(posStamp):
	arr = np.zeros(6)
	arr[0] = posStamp.pose.position.x
	arr[1] = posStamp.pose.position.y
	arr[2] = posStamp.pose.position.z
	quat = [posStamp.pose.orientation.x, posStamp.pose.orientation.y, posStamp.pose.orientation.z, posStamp.pose.orientation.w]
	euler = euler_from_quaternion(quat)
	arr[3] = euler[0]
	arr[4] = euler[1]
	arr[5] = euler[2]
	return arr

def execute_cart_path(arm, waypoints):
	res = False
	maxtries = 200
	attempts = 0
	fraction = 0.0

	while fraction < 1.0 and attempts < maxtries:
		(plan, fraction) = arm.group[0].compute_cartesian_path(
								 waypoints,   # waypoints to follow
								 0.025,        # eef_step
								 0.0)         # jump_threshold

		# Increment the number of attempts
		attempts += 1

		# Print out a progress message
		if attempts % 10 == 0:
		 rospy.loginfo("Still trying after " + str(attempts) + " attempts...")

	# If we have a complete plan, execute the trajectory
	if fraction == 1.0:
		plan = simplify_plan(plan, arm)
		print 'plan =', plan
		rospy.loginfo("Path computed successfully. Moving the arm.")
		raw_input('Going to execute plan, Press Enter if proceed')
		res = arm.group[0].execute(plan)
		rospy.loginfo("Path execution complete.")
	else:
		rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")


def main():
	global joint_traj
	arm = ArmMoveIt(planning_frame='right_link_base', _arm_name='right')

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
	for i in range(1):
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
	waypoints = []
	ee_traj = []

	for i, pt in enumerate(joint_traj_saved.points):
		ee_pose = arm.get_FK(root='right_link_base', req_frame='right_link_7', joints=pt.positions)[0]
		ee_pose.header.seq = i
		ee_pose.header.stamp = pt.time_from_start.to_sec()
		ee_traj.append(copy.deepcopy(ee_pose))
		waypoints.append(copy.deepcopy(ee_pose.pose))

	# execute_cart_path(arm, waypoints)

	print 'ee_traj =', ee_traj

	# save it
	rospack = rospkg.RosPack()
	cwd = rospack.get_path('gemini_tutorials')
	filename = cwd + '/src/ee_trajectory_frame_right_link_base.pickle'
	file = open(filename,'wb')
	pickle.dump(ee_traj, file) 
	file.close()

	# Playback tracjectory

	'''
	Depricated
	#for ee_pose in ee_traj:
	#	arm.move_to_ee_pose(ee_pose.pose)

	# print 'Going to the first ee pose in ee traj'
	# joint_traj = arm._ee_traj_to_joint_traj(ee_traj)
	# arm.move_to_joint_pose(joint_traj.points[0].positions)
	# print 'Exercuting the rest traj'
	# arm.execute_joint_trajectory(joint_traj)

	# arm.execute_ee_trajectory(ee_traj, use_custom_time=False)
	#joint_traj_saved = tweak_time(joint_traj_saved, 2)
	# arm.execute_joint_trajectory(joint_traj_saved, use_custom_time=True)
	
	# action_goal = FollowJointTrajectoryGoal()
	# action_goal.trajectory = 
	'''
	raw_input('Playing back the trajectory, Press Enter!')

	urdf = 'package://cartesian_trajectory_planner/urdf/jaco_7dof.urdf'
	srdf = 'package://cartesian_trajectory_planner/urdf/jaco_7dof.srdf'

	cart_planner = TrajOptCartesianPlanner(urdf, srdf, viz=False)
	current_pose = arm.get_current_pose()
	cart_planner.setDOFs(current_pose)
	print "start", start
	print "current_pose", current_pose

	cal_traj = cart_planner.plan('right_arm', ee_traj, link='right_link_7')
	print "cal_traj ", cal_traj
	joint_traj = joint_array_to_traj(cal_traj.tolist())

	arm.execute_joint_trajectory(joint_traj)

	# raw_input('Correcting for the goal state. Press Enter!')
	# jointTarget = end
	# arm.move_to_joint_pose(jointTarget)
	# rospy.sleep(5)




	# Shutdown
	print "shutting down!"
	# shut down moveit
	moveit_commander.roscpp_shutdown()
	moveit_commander.os._exit(0)




if __name__=='__main__':
	rospy.init_node('demo_manipulation', anonymous=True)
	main()
	# rospy.spin()
	
	
	
	
	
