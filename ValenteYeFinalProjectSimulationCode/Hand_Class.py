#!/usr/bin/python

"""
Sample Solution for PA2
Use "run.py [--sim] pa2_solution" to execute
"""
import math
import numpy as np


class Hand:
	# def __init__(self, arm):
	def __init__(self, factory):
		"""Constructor.

		Args:
			factory (factory.FactorySimulation)
		"""
		# self.arm = factory.create_kuka_lbr4p()
		# self.time = factory.create_time_helper()
		self.arm = factory.create_kuka_lbr4p()
		self.time = factory.create_time_helper()
		self.create = factory.create_create()

	def forward_kinematics(self, theta1, theta2):
		self.arm.go_to(1, theta1)
		self.arm.go_to(3, theta2)
		L1 = 0.4  # estimated using V-REP (joint2 - joint4)
		L2 = 0.39  # estimated using V-REP (joint4 - joint6)
		z = L1 * math.cos(theta1) + L2 * math.cos(theta1 + theta2) + 0.3105
		x = L1 * math.sin(theta1) + L2 * math.sin(theta1 + theta2)
		print("Go to {},{} deg, FK: [{},{},{}]".format(math.degrees(theta1), math.degrees(theta2), -x, 0, z))

	def inverse_kinematics(self, x_i, z_i):
		L1 = 0.4  # estimated using V-REP (joint2 - joint4)
		L2 = 0.39  # estimated using V-REP (joint4 - joint6)
		# Corrections for our coordinate system
		z = z_i - 0.3105
		x = -x_i
		# compute inverse kinematics
		r = math.sqrt(x * x + z * z)
		print((L1 * L1 + L2 * L2 - r * r) / (2 * L1 * L2))
		alpha = math.acos((L1 * L1 + L2 * L2 - r * r) / (2 * L1 * L2))
		theta2 = math.pi - alpha

		beta = math.acos((r * r + L1 * L1 - L2 * L2) / (2 * L1 * r))
		theta1 = math.atan2(x, z) - beta
		if theta2 < -math.pi / 2.0 or theta2 > math.pi / 2.0 or theta1 < -math.pi / 2.0 or theta1 > math.pi / 2.0:
			theta2 = math.pi + alpha
			theta1 = math.atan2(x, z) + beta
		if theta2 < -math.pi / 2.0 or theta2 > math.pi / 2.0 or theta1 < -math.pi / 2.0 or theta1 > math.pi / 2.0:
			print("Not possible")
			return

		self.arm.go_to(1, theta1)
		self.arm.go_to(3, theta2)
		print("Go to [{},{}], IK: [{} deg, {} deg]".format(x_i, z_i, math.degrees(theta1), math.degrees(theta2)))

	def putOnShelf3(self):
		self.arm.open_gripper()
		self.create.drive_direct(0, 0)
		for x in range(73):
			self.arm.go_to(1, math.radians(x))
			self.time.sleep(0.1)
		for x in range(42):
			self.arm.go_to(3, math.radians(x))
			self.time.sleep(0.1)
		self.time.sleep(5)
		print("located")
		self.arm.open_gripper()
		self.arm.close_gripper()
		self.time.sleep(15)

		for x in range(65):
			halfx = x / 2
			self.time.sleep(0.1)
			self.forward_kinematics(math.radians(100 - x), math.radians(halfx))
		self.time.sleep(5)
		for x in range(85):
			self.time.sleep(0.1)
			self.arm.go_to(0, math.radians(-x * 1.5))

		self.time.sleep(4)
		self.arm.open_gripper()
		self.time.sleep(20)

	def putOnShelf2(self):
		self.arm.open_gripper()
		self.create.drive_direct(0, 0)
		for x in range(73):
			self.arm.go_to(1, math.radians(x))
			self.time.sleep(0.1)
		for x in range(42):
			self.arm.go_to(3, math.radians(x))
			self.time.sleep(0.1)
		self.time.sleep(5)
		print("located")
		self.arm.open_gripper()
		self.arm.close_gripper()
		self.time.sleep(15)
		for x in range(25):
			self.forward_kinematics(math.radians(100 - x), math.radians(0))
			self.time.sleep(0.1)
		self.time.sleep(5)
		for x in range(85):
			self.time.sleep(0.1)
			self.arm.go_to(0, math.radians(-x * 1.5))
		self.time.sleep(4)
		self.arm.open_gripper()
		self.time.sleep(15)