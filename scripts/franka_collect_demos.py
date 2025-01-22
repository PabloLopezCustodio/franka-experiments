#!/usr/bin/env python3

import numpy as np
import time
import os
import json
import sys
from math import pi
import random
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import ErrorRecoveryAction, ErrorRecoveryActionGoal, FrankaState
from moveit_msgs.msg import Constraints, OrientationConstraint, JointConstraint


N_DEMOS = 30 # number of demonstrations to be collected
RAND_START = True # randomise the start configuration of the robot for each demonstration
SAVE_FRAME_8 = False # save the pose frame 8 instead of frame E.

class FrankaRobot(object):
	def __init__(self):
		super(FrankaRobot, self).__init__()
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('FrankaRobot', anonymous=True, disable_signals=True)
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


if __name__ == '__main__':
	# Moveit's end-effector frame is set to 'panda_link8'
	# Libfranka's end-effector frame is 'panda_EE'
	# O_T_EE in franka_states topic uses 'panda_EE'  

	robot = FrankaRobot()

	# Reset controller in case there was any problem in the previous execution
	input("make sure External Activation button is disabled")
	robot.reset_controller()

	# GO TO HOME POSITION
	robot.go_home()

	# Joint values at start position
	joint_start = np.array([0.0,
							-0.6477529998454954,
							0.0,
							-2.442111407932482,
							0.0,
							1.8335964316287736,
							0.785398])

	# Transformation between frames E and 8: Pose of frame 8 with respoect to frame E
	TE8 = np.array([[0.707106781, -0.707106781, 0.0, 0.0],
					[0.707106781, 0.707106781, 0.0, 0.0],
					[0.0, 0.0, 1.0, -0.1034],
					[0.0, 0.0, 0.0, 1.0]])

	for demo in range(N_DEMOS):
		if RAND_START:
			robot.go_to_joint_state(joint_start + np.random.normal(0, 0.1, 7)) # maximum disturbance is 0.1 radians
		else:
			robot.go_to_joint_state(joint_start)
		print('initial configuration:', robot.robot_state.q)
		states = []
		input("Press Enter to start, \nthen activate External Activation button")
		time.sleep(4)
		print('\n\n')
		print('================================')
		print('      R E C O R D I N G')
		print('================================')
		print("\n\nPress Ctrl+C to stop recording")
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
			TOE = np.reshape(states[j].O_T_EE, (4, 4))
			TOE = TOE.transpose()
			if SAVE_FRAME_8:
				poses.append(np.matmul(TOE, TE8).tolist())
			else:
				poses.append(TOE.tolist())
			joints.append(states[j].q)
			times.append(states[j].header.stamp.secs + states[j].header.stamp.nsecs*1e-9)
		with open(os.path.join("demonstrations", f"trajectory_{demo+1}.json"), "w") as outfile:
			json.dump({"TOE_traj": poses,
					   "joints_traj": joints,
					   "timestamps": times}, outfile)
		print(f"\n\ndemostration {demo+1} saved \n")
		input("deactivate External Activation button \npress Enter")
		# reset robot mode
		robot.reset_controller()
