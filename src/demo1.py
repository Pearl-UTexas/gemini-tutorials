#!/usr/bin/env python 

import rospy
import std_msgs
import vector_msgs
import geometry_msgs.msg
from hlpr_manipulation_utils.arm_moveit2 import *
from hlpr_manipulation_utils.manipulator import Gripper

def myhook():
  	print "shutting down!"
	# shut down moveit
	moveit_commander.roscpp_shutdown()
	moveit_commander.os._exit(0)


def main():
	pan_publisher = rospy.Publisher('/pan_controller/command', std_msgs.msg.Float64, queue_size=10)
	tilt_publisher = rospy.Publisher('/tilt_controller/command', std_msgs.msg.Float64, queue_size=10)
	linear_actuator_publisher = rospy.Publisher('/vector/linear_actuator_cmd', vector_msgs.msg.LinearActuatorCmd, queue_size=10)
	arm = ArmMoveIt(planning_frame='linear_actuator_link', _arm_name='right')
	gripper = Gripper(prefix='right')

	# Move Pan
	raw_input('Executing Pan command. Press Enter!')
	pan_angle = std_msgs.msg.Float64()
	pan_angle.data = 0.0
	pan_publisher.publish(pan_angle)
	rospy.sleep(2)

	pan_angle.data = -1.0
	pan_publisher.publish(pan_angle)
	rospy.sleep(3)

	# Move tilt
	raw_input('Executing tilt command. Press Enter!')
	tilt_angle = std_msgs.msg.Float64()
	tilt_angle.data = 0.0
	tilt_publisher.publish(tilt_angle)
	rospy.sleep(3)

	tilt_angle.data = 0.5
	tilt_publisher.publish(tilt_angle)
	rospy.sleep(3)

	# move linear_actuator
	raw_input('Executing linear joint command. Press Enter!')
	height = vector_msgs.msg.LinearActuatorCmd()
	height.desired_position_m = 0.55
	linear_actuator_publisher.publish(height)
	rospy.sleep(5)

	height.desired_position_m = 0.45
	linear_actuator_publisher.publish(height)
	rospy.sleep(5)

	#Move arm to a joint target
	# print arm.group[0].get_joints()
	raw_input('Executing Arm trajectory command. Press Enter!')
	jointTarget = [1.689, 4.739, 0.118, 4.313, -0.148, 1.150, 3.340]
	arm.move_to_joint_pose(jointTarget)
	rospy.sleep(3)


	#Open gripper
	gripper.open(100)
	rospy.sleep(3)

	#Close gripper
	gripper.close(100)
	rospy.sleep(3)

	#Move arm to a cartesian target
	tarPose = geometry_msgs.msg.Pose()
	tarPose.position.x = 0.741
	tarPose.position.y = -0.245
	tarPose.position.z = 0.255
	tarPose.orientation.x = -0.631
	tarPose.orientation.y = 0.304
	tarPose.orientation.z = 0.398
	tarPose.orientation.w = 0.592

	arm.move_to_ee_pose(tarPose)
	rospy.sleep(3)

	#Open gripper
	gripper.open(100)
	rospy.sleep(3)

	# Homing
	jointTarget = [1.689, 4.739, 0.118, 4.313, -0.148, 1.150, 3.340]
	arm.move_to_joint_pose(jointTarget)
	rospy.sleep(3)

	# Shutdown
	rospy.on_shutdown(myhook)




if __name__=='__main__':
	rospy.init_node('demo_manipulation', anonymous=True)
	main()
	# rospy.spin()
	
	
	
	
	
