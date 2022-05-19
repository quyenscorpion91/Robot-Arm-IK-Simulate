from array import array
from cgitb import text
from functools import partial
from re import T
from typing import ChainMap
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
import math
import time
import kinematic.kinematic as kine
import trajectory.trajectory as traj

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
	trajec = traj.Trajectory

	preAngle = [0,0,0,0,0,0]
	# currentAngle = [0,0,0,0,0,0]
	kine.ArmPos.anglerad = [0, 0, kine.PI/2, 0, 0, 0]
	# ArmPos.anglerad = [0, 0, 0, 0, 0, 0]
	# 200, 200, 100
	# ArmPos.anglerad = [0.6995645729204552, 1.9223789555686266, -1.1647433216113514, -0.2669742700544176, -0.784643352953723, 0.3794174161304081]

	for i in range(kine.NUM_ANGLE):
		preAngle[i] = kine.ArmPos.anglerad[i]

	last = armKine.ForwardKinematicSim(armKine ,True)

	# ax.plot(kine.ArmPos.x, kine.ArmPos.y, kine.ArmPos.z, label='Arm')
	# ax.legend()

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
	armKine.InverseKinematicSim(armKine, 400.0, 300.0, 500.0, kine.PI/4, kine.PI/2, kine.PI/6)
	last = armKine.ForwardKinematicSim(armKine, True)

	checkAngle = armKine.GetRotateAngleFromMatrix(armKine, last)
	print("curent rotate: " + str(checkAngle))
	print("curent DOF: " + str(kine.ArmPos.anglerad))
	print("====================== COODINATE HERE =====================================")
	print("X: " + str(kine.ArmPos.x))
	print("Y: " + str(kine.ArmPos.y))
	print("Z: " + str(kine.ArmPos.z))
	ax.plot(kine.ArmPos.x, kine.ArmPos.y, kine.ArmPos.z, label='IK')
	ax.legend()

	# armKine.InverseKinematicSimNew(armKine, 400.0, 300.0, 500.0, 0.0, kine.PI/2, 0.0)
	# last = armKine.ForwardKinematicSim(armKine, True)

	# checkAngle = armKine.GetRotateAngleFromMatrix(armKine, last)
	# print("curent rotate: " + str(checkAngle))
	# print("curent DOF: " + str(kine.ArmPos.anglerad))
	# print("====================== COODINATE HERE =====================================")
	# print("X: " + str(kine.ArmPos.x))
	# print("Y: " + str(kine.ArmPos.y))
	# print("Z: " + str(kine.ArmPos.z))
	# ax.plot(kine.ArmPos.x, kine.ArmPos.y, kine.ArmPos.z, label='IK')
	# ax.legend()
	
	# qarr = [[11,12,13,14],[21,22,23,24],[31,32,33,34],[41,42,43,44]]
	# qarr = np.array(qarr)
	# qsarr = qarr[:3,:3]
	# print("qarr: " + str(qarr))
	# print("qsarr " + str(qsarr))

	# simulate for rotation
	# targetAngle = [0.0, 3.49066, 1.5708]
	# print("target rotate: " + str(targetAngle))
	# diff = [0.0, 0.0, 0.0]
	# diff[0] = abs(targetAngle[0] - checkAngle[0])
	# diff[1] = abs(targetAngle[1] - checkAngle[1])
	# diff[2] = abs(targetAngle[2] - checkAngle[2])
	# maxDiff = max(diff)
	# iter = int(maxDiff / 0.1) + 1
	# print("iter: " + str(iter))
	# print("diff: " + str(diff))
	# iterDiff = [0.0, 0.0, 0.0]
	# iterDiff[0] = diff[0] / float(iter)
	# iterDiff[1] = diff[1] / float(iter)
	# iterDiff[2] = diff[2] / float(iter)
	# currentAngle = checkAngle
	# print("current angle: " + str(currentAngle))
	# print("iterDiff: " + str(iterDiff))
	# for i in range(iter):
	# 	for j in range(3):
	# 		currentAngle[j] += iterDiff[j]
	# 	armKine.InverseKinematicSim(armKine, 400.0, 300.0, 500.0, currentAngle[0], currentAngle[1], currentAngle[2])
	# 	last = armKine.ForwardKinematicSim(armKine, True)
	# 	ax.plot(kine.ArmPos.x, kine.ArmPos.y, kine.ArmPos.z, label='IK' + str(i))
	# 	ax.legend()

	# checkAngle = armKine.GetRotateAngleFromMatrix(armKine, last)
	# print("curent rotate: " + str(checkAngle))
	# print("current pos: " + str(kine.ArmPos.x[8]) + " " + str(kine.ArmPos.y[8]) + " " + str(kine.ArmPos.z[8]))
	# print("DOF: " + str(kine.ArmPos.anglerad))
	# ====================== done =============================
	
	# simulate for linear
	# point = trajec.BresenhamLinear(400.0, 300.0, 500.0, 300.0, 500.0, 400.0)
	# print("len(point.x): " + str(len(point.x)))
	# ax.plot(point.x, point.y, point.z, label='line')
	# ax.legend()

	# for i in range (int(len(point.x)/10)):
	# 	armKine.InverseKinematicSim(armKine, point.x[i*10], point.y[i*10], point.z[i*10], 0.0, 3.49066, 0.0)
	# 	last = armKine.ForwardKinematicSim(armKine, True)
	# 	ax.plot(kine.ArmPos.x, kine.ArmPos.y, kine.ArmPos.z, label='IK' + str(i))
	# 	ax.legend()
	
	# # draw last point
	# armKine.InverseKinematicSim(armKine, point.x[len(point.x) - 1], point.y[len(point.x) - 1], point.z[len(point.x) - 1], 0.0, 3.49066, 0.0)
	# last = armKine.ForwardKinematicSim(armKine, True)
	# ax.plot(kine.ArmPos.x, kine.ArmPos.y, kine.ArmPos.z, label='IK' + str(i))
	# ax.legend()
	
	# checkAngle = armKine.GetRotateAngleFromMatrix(armKine, last)
	# print("curent rotate: " + str(checkAngle))
	# print("current pos: " + str(kine.ArmPos.x[8]) + " " + str(kine.ArmPos.y[8]) + " " + str(kine.ArmPos.z[8]))
	# print("DOF: " + str(kine.ArmPos.anglerad))
	# ====================== done =============================
	
	# simulate for circle
	# point = trajec.BresenhamCircle(400.0, 200.0, 500.0, 100.0, 0) # first 1/8 circle
	# point1 = trajec.BresenhamCircle(400.0, 200.0, 500.0, 100.0, 1) # second 1/8 circle
	# ax.legend()

	# pointLen = len(point.x)
	# # merged circle
	# for i in range(len(point.x)):
	# 	point.x.append(point1.x[pointLen - i - 1])
	# 	point.y.append(point1.y[pointLen - i - 1])
	# 	point.z.append(point1.z[pointLen - i - 1])

	# ax.plot(point.x, point.y, point.z, label='IK' + str(i))
	# ax.legend()

	# for i in range (int(len(point.x)/10)):
	# 	armKine.InverseKinematicSim(armKine, point.x[i*10], point.y[i*10], point.z[i*10], 0.0, 3.49066, 0.0)
	# 	last = armKine.ForwardKinematicSim(armKine, True)
	# 	ax.plot(kine.ArmPos.x, kine.ArmPos.y, kine.ArmPos.z, label='IK' + str(i))
	# 	ax.legend()
	
	# # draw last point
	# armKine.InverseKinematicSim(armKine, point.x[len(point.x) - 1], point.y[len(point.x) - 1], point.z[len(point.x) - 1], 0.0, 3.49066, 0.0)
	# last = armKine.ForwardKinematicSim(armKine, True)
	# ax.plot(kine.ArmPos.x, kine.ArmPos.y, kine.ArmPos.z, label='IK' + str(i))
	# ax.legend()
	
	# checkAngle = armKine.GetRotateAngleFromMatrix(armKine, last)
	# print("curent rotate: " + str(checkAngle))
	# print("current pos: " + str(kine.ArmPos.x[8]) + " " + str(kine.ArmPos.y[8]) + " " + str(kine.ArmPos.z[8]))
	# print("DOF: " + str(kine.ArmPos.anglerad))
	# ====================== done =============================
	

	plt.show()
