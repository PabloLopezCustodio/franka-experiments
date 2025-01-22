#!/usr/bin/env python3

from __future__ import division
import numpy as np
import math
import matplotlib.pyplot as plt
import numpy.matlib as mat
from numpy.linalg import inv
import time


import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import PoseStamped

from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from tf.transformations import quaternion_from_euler
import random

from franka_msgs.msg import ErrorRecoveryAction, ErrorRecoveryActionGoal, FrankaState
from moveit_msgs.msg import Constraints, OrientationConstraint, JointConstraint




class FrankaRobot(object):
	def __init__(self):
		super(FrankaRobot, self).__init__()
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('FrankaRobotWorkshop', anonymous=True, disable_signals=True)
		self.robot = moveit_commander.RobotCommander()
		self.move_group = moveit_commander.MoveGroupCommander("panda_arm")
		self.move_group.set_end_effector_link("panda_link8")
		self.move_group.set_planner_id("RRTConnectkConfigDefault")
		self.move_group.set_max_velocity_scaling_factor(1)
		self.move_group.set_max_acceleration_scaling_factor(1)
		self.constraints = Constraints()
		self.pub_recover = rospy.Publisher('/franka_control/error_recovery/goal', 
										ErrorRecoveryActionGoal, 
										queue_size=10)
		self.sub_state = rospy.Subscriber('/franka_state_controller/franka_states', 
										FrankaState, 
										self.state_callback)
		self.robot_state = None


	def get_robot_joint_state(self):
		robot_state = self.robot.get_current_state().joint_state.position
		return robot_state

	def get_robot_task_state(self):
		robot_ee_pose = self.move_group.get_current_pose().pose
		return robot_ee_pose

	def go_to_joint_state(self, joint_goal):
		self.move_group.go(joint_goal, wait=True)
		self.move_group.stop() 

	def go_to_task_state(self,position,quaternion):
		p = PoseStamped()
		p.header.frame_id = '/panda_link0'
		p.pose.position.x = position[0]
		p.pose.position.y = position[1]
		p.pose.position.z = position[2]
		p.pose.orientation.x = quaternion[0]
		p.pose.orientation.y = quaternion[1]
		p.pose.orientation.z = quaternion[2]
		p.pose.orientation.w = quaternion[3]
		target = self.move_group.set_pose_target(p)
		self.move_group.go(target)
		self.move_group.stop()
		#robot_ee_pose = self.move_group.get_current_pose().pose
		#print("final POSE:")
		#print(robot_ee_pose)

	def constrain_eef(self, ogoal):
		orientation_constraint = OrientationConstraint()
		orientation_constraint.link_name = robot.move_group.get_end_effector_link()
		orientation_constraint.header.frame_id = 'panda_link0'
		orientation_constraint.orientation.x = ogoal[0]
		orientation_constraint.orientation.y = ogoal[1]
		orientation_constraint.orientation.z = ogoal[2]
		orientation_constraint.orientation.w = ogoal[3]
		orientation_constraint.absolute_x_axis_tolerance = 0.05
		orientation_constraint.absolute_y_axis_tolerance = 0.05
		orientation_constraint.absolute_z_axis_tolerance = 0.05
		orientation_constraint.weight = 1.0
		self.constraints.orientation_constraints = [orientation_constraint]
		self.move_group.set_path_constraints(self.constraints)

	def clear_constraints(self):
		self.move_group.clear_path_constraints()
		self.constraints.joint_constraints = []
		self.constraints.orientation_constraints = []

	def constrain_joint(self, joint_name, value, tolerance):
		# joint names start at panda_joint1
		if not(value):
			value = self.get_robot_joint_state()[int(joint_name[-1])-1]
		joint_constraint = JointConstraint()
		joint_constraint.joint_name = joint_name
		joint_constraint.position = value
		joint_constraint.tolerance_above = tolerance
		joint_constraint.tolerance_below = tolerance
		joint_constraint.weight = 1.0
		self.constraints.joint_constraints.append(joint_constraint)
		self.move_group.set_path_constraints(self.constraints)		


	def go_home(self):
		# joint space home:
		goodjoints = [0.0,
				-0.6477529998454954,
				0.0,
				-2.442111407932482,
				0.0,
				1.8335964316287736,
				0.785398]
		self.move_group.go(goodjoints, wait=True)
		self.move_group.stop()
		# cartesian home:
		ogoal = [0.9238795, -0.3826834, 0, 0]
		pgoal = [0.40, 0.0, 0.70]
		self.move_group.clear_path_constraints()
		self.constrain_joint('panda_joint1', 0.0, 5.0*pi/180.0)
		robot.go_to_task_state(pgoal, ogoal)
		self.clear_constraints()

	def state_callback(self, msg):
		self.robot_state = msg

	def reset_controller(self):
		print('recovering...')
		rate = rospy.Rate(10) # 10hz
		# publish during 3 seconds as recommended
		for i in range(0, 40):
			self.pub_recover.publish(ErrorRecoveryActionGoal())
			rate.sleep()
		while self.robot_state.robot_mode != 2:
			pass
		print('\n robot restored to mode %s\n' % self.robot_state.robot_mode)

#def listen2franka_states():
#    return rospy.wait_for_message('/franka_state_controller/franka_states', FrankaState)




if __name__ == '__main__':

	# Moveit's end-effector frame is set to 'panda_link8'
	# Libfranka's end-effector frame is 'panda_EE' (I can't get Moveit use this one without the gripper)
	# O_T_EE in franka_states topic uses 'panda_EE'  

	robot = FrankaRobot()

	# GO TO HOME POSITION
	robot.go_home()

	# Joint values at start position
	joint_start = [-0.13881318235431997,
					0.9670546870624386,
					0.6110642413654681,
					-1.6666014525597055,
					-0.5043063486781384,
					2.5394160579707887,
					1.4077132147865163]

	NoDemos = 30
	for demo in range(0, NoDemos):
		robot.go_to_joint_state(joint_start)
		states = []
		input("Press Enter to start, \nthen activate S-button. \nPress Ctrl+C to terminate")
		time.sleep(4)
		print('\n\n')
		print('=======================')
		print('   R E C O R D I N G')
		print('=======================')
		print('\n\n')
		try:
			while True:
				states.append(robot.robot_state)
				time.sleep(0.05)
		except KeyboardInterrupt:
			pass
		times = []
		joints = []
		poses = []
		for j in range(0, len(states)):
			poses.append(states[j].O_T_EE)
			joints.append(states[j].q)
			times.append(states[j].header.stamp.secs + states[j].header.stamp.nsecs*1e-9)
		a = np.array(poses)
		file_path = ('/home/pablolopezcustodio/pc_ws/src/teach/demonstrations/POSES%s.csv' % (demo+121))
		np.savetxt(file_path, a, delimiter=",")
		a = np.array(joints)
		file_path = ('/home/pablolopezcustodio/pc_ws/src/teach/demonstrations/JOINTS%s.csv' % (demo+121))
		np.savetxt(file_path, a, delimiter=",")
		a = np.array(times) - times[0]
		file_path = ('/home/pablolopezcustodio/pc_ws/src/teach/demonstrations/TIMES%s.csv' % (demo+121))
		np.savetxt(file_path, a, delimiter=",")
		print("\n\ndemostration %s saved \n" % (demo))
		input("deactivate S-button \npress Enter")
		# reset robot mode
		robot.reset_controller()