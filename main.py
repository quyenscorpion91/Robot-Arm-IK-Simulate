from functools import partial
from typing import ChainMap
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
import math
import time
import kinematic.kinematic as kine

from numpy.lib.function_base import kaiser


if __name__ == '__main__':
	mpl.rcParams['legend.fontsize'] = 10

	print("Quyen Truong simulate arm robot 6 DOF")

	fig = plt.figure()
	ax = fig.gca(projection='3d')
	plt.xlim(-100, 500)
	plt.ylim(-100, 500)
	plt.ylim(-100, 500)

	armKine = kine.Kinematic


	# ArmPos.anglerad = [0, 0, 0, 0, 0, 0]
	# ForwardKinematicSim()

	# ax.plot(ArmPos.x, ArmPos.y, ArmPos.z, label='IK')
	# ax.legend()

	# print("Arm grad: " + str(ArmPos.anglerad))

	# print("X: " + str(ArmPos.x))
	# print("Y: " + str(ArmPos.y))
	# print("Z: " + str(ArmPos.z))

	# ArmPos.anglerad = [PI/9, PI/4, PI/4, 0, -PI/4, 0]
	# ForwardKinematicSim()

	# ax.plot(ArmPos.x, ArmPos.y, ArmPos.z, label='IK')
	# ax.legend()

	# print("Arm grad: " + str(ArmPos.anglerad))

	# print("X: " + str(ArmPos.x))
	# print("Y: " + str(ArmPos.y))
	# print("Z: " + str(ArmPos.z))

	# ArmPos.anglerad = [PI/8, PI/5, PI/5, 0, -PI/5, 0]
	# ForwardKinematicSim()

	# ax.plot(ArmPos.x, ArmPos.y, ArmPos.z, label='IK')
	# ax.legend()

	# print("Arm grad: " + str(ArmPos.anglerad))

	# print("X: " + str(ArmPos.x))
	# print("Y: " + str(ArmPos.y))
	# print("Z: " + str(ArmPos.z))

	# ArmPos.anglerad = [PI/7, PI/6, PI/6, 0, -PI/6, 0]
	# ForwardKinematicSim()

	# ax.plot(ArmPos.x, ArmPos.y, ArmPos.z, label='IK')
	# ax.legend()

	# print("Arm grad: " + str(ArmPos.anglerad))

	# print("X: " + str(ArmPos.x))
	# print("Y: " + str(ArmPos.y))
	# print("Z: " + str(ArmPos.z))

	# ArmPos.anglerad = [PI/6, PI/7, PI/7, 0, -PI/7, 0]
	# ForwardKinematicSim()

	# ax.plot(ArmPos.x, ArmPos.y, ArmPos.z, label='IK')
	# ax.legend()

	# print("Arm grad: " + str(ArmPos.anglerad))

	# print("X: " + str(ArmPos.x))
	# print("Y: " + str(ArmPos.y))
	# print("Z: " + str(ArmPos.z))

	# ArmPos.anglerad = [PI/5, PI/8, PI/8, 0, -PI/8, 0]
	# ForwardKinematicSim()

	# ax.plot(ArmPos.x, ArmPos.y, ArmPos.z, label='IK')
	# ax.legend()

	# print("Arm grad: " + str(ArmPos.anglerad))

	# print("X: " + str(ArmPos.x))
	# print("Y: " + str(ArmPos.y))
	# print("Z: " + str(ArmPos.z))
	preAngle = [0,0,0,0,0,0]
	currentAngle = [0,0,0,0,0,0]
	kine.ArmPos.anglerad = [0, kine.PI/4, kine.PI/4, 0, 0, 0]
	# ArmPos.anglerad = [0, 0, 0, 0, 0, 0]
	# 200, 200, 100
	# ArmPos.anglerad = [0.6995645729204552, 1.9223789555686266, -1.1647433216113514, -0.2669742700544176, -0.784643352953723, 0.3794174161304081]

	for i in range(kine.NUM_ANGLE):
		preAngle[i] = kine.ArmPos.anglerad[i]

	last = armKine.ForwardKinematicSim(armKine ,True)

	ax.plot(kine.ArmPos.x, kine.ArmPos.y, kine.ArmPos.z, label='Arm')
	ax.legend()

	print("Current Arm status: " + str(kine.ArmPos.anglerad))

	print("===========================================================")
	print("Current Arm position: ")
	print("X: " + str(kine.ArmPos.x[8]))
	print("Y: " + str(kine.ArmPos.y[8]))
	print("Z: " + str(kine.ArmPos.z[8]))

	print("====================== COODINATE HERE =====================================")
	print("X: " + str(kine.ArmPos.x))
	print("Y: " + str(kine.ArmPos.y))
	print("Z: " + str(kine.ArmPos.z))

	checkAngle = armKine.GetRotateAngleFromMatrix(armKine, last)
	print("curent rotate: " + str(checkAngle))

	print("===========================================================")
	target = [400.0, 350.0, 500.0]
	print("Target Position: " + str(target))
	armKine.InverseKinematicSim(armKine, target[0], target[1], target[2], kine.PI/20, kine.PI/30, 0)
	# armKine.InverseKinematicSimNew(armKine, target[0], target[1], target[2], 0, kine.PI/2, 0)
	# armKine.InverseKinematicSimNewest(armKine, target[0], target[1], target[2], 0, kine.PI/2, 0)

	ret = armKine.ForwardKinematicSim(armKine, True)

	# ax.plot(kine.ArmPos.x, kine.ArmPos.y, kine.ArmPos.z, label='IK')
	# ax.legend()

	# print("Current Arm status: " + str(ArmPos.anglerad))

	# print("===========================================================")
	# print("Current Arm position: ")
	# print("X: " + str(ret))
	# ret = armKine.ForwardKinematicSim(armKine, False)
	# print("Current Arm J7 position: ")
	# print("X: " + str(ret))

	errX = abs(target[0] - kine.ArmPos.x[8])
	errY = abs(target[1] - kine.ArmPos.y[8])
	errZ = abs(target[2] - kine.ArmPos.z[8])

	print("Calculate Err: ")
	print("X: " + str(errX))
	print("Y: " + str(errY))
	print("Z: " + str(errZ))

	# draw a simulate
	for i in range(kine.NUM_ANGLE):
		currentAngle[i] = kine.ArmPos.anglerad[i]

	iter = [0.0,0.0,0.0,0.0,0.0,0.0]
	for i in range(kine.NUM_ANGLE):
		iter[i] = (currentAngle[i] - preAngle[i]) / 10.0

	kine.ArmPos.anglerad = [preAngle[0],preAngle[1],preAngle[2],preAngle[3],preAngle[4],preAngle[5]]
	for i in range(10):
		for j in range(kine.NUM_ANGLE):
			kine.ArmPos.anglerad[j] += iter[j]

		armKine.ForwardKinematicSim(armKine, True)
		ax.plot(kine.ArmPos.x, kine.ArmPos.y, kine.ArmPos.z, label='IK')
		ax.legend()

	# for i in range(kine.NUM_ANGLE):
	# 	if abs(kine.ArmPos.anglerad[i]) > kine.PI:
	# 		kine.ArmPos.anglerad[i] = kine.ArmPos.anglerad[i] % kine.PI
	# 	print("anglegrad[" + str(i) + "] = " + str(kine.ArmPos.anglerad[i]))
	# draw a latest
	last = armKine.ForwardKinematicSim(armKine, True)
	ax.plot(kine.ArmPos.x, kine.ArmPos.y, kine.ArmPos.z, label='IK')
	ax.legend()

	checkAngle = armKine.GetRotateAngleFromMatrix(armKine, last)
	print("curent rotate: " + str(checkAngle))

	# delta = 0.0001
	# err = [0.0, 0.0, 0.0]
	# for i in range(100000):
	# 	last = armKine.ForwardKinematicSim(armKine, True)
	# 	checkAngle = armKine.GetRotateAngleFromMatrix(armKine, last)
	# 	err[0] = abs(0 - checkAngle[0])
	# 	err[1] = abs(-kine.PI/4 - checkAngle[1])
	# 	err[2] = abs(0 - checkAngle[2])
	# 	ttErr = (err[0] + err[1] + err[2]) / 3.0
	# 	if ttErr < 0.1:
	# 		break
	# 	kine.ArmPos.anglerad[5] += delta

	# last = armKine.ForwardKinematicSim(armKine, True)
	# checkAngle = armKine.GetRotateAngleFromMatrix(armKine, last)
	# print("curent rotate: " + str(checkAngle))

	plt.show()
