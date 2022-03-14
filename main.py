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
	kine.ArmPos.anglerad = [kine.PI/4, kine.PI/4, kine.PI/4, 0, 0, 0]
	# ArmPos.anglerad = [0, 0, 0, 0, 0, 0]
	# 200, 200, 100
	# ArmPos.anglerad = [0.6995645729204552, 1.9223789555686266, -1.1647433216113514, -0.2669742700544176, -0.784643352953723, 0.3794174161304081]

	for i in range(kine.NUM_ANGLE):
		preAngle[i] = kine.ArmPos.anglerad[i]

	armKine.ForwardKinematicSim(armKine ,True)

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

	print("===========================================================")
	target = [200, 200, 300]
	print("Target Position: " + str(target))
	armKine.InverstKinematicSim(armKine, target[0], target[1], target[2], 0, kine.PI/4, 0)
	# armKine.InverstKinematicSimNew(armKine, target[0], target[1], target[2], 0, -kine.PI/4, 0)

	ret = armKine.ForwardKinematicSim(armKine, True)

	# ax.plot(ArmPos.x, ArmPos.y, ArmPos.z, label='IK')
	# ax.legend()

	# print("Current Arm status: " + str(ArmPos.anglerad))

	# print("===========================================================")
	# print("Current Arm position: ")
	# print("X: " + str(ret))
	ret = armKine.ForwardKinematicSim(armKine, False)
	# print("Current Arm J7 position: ")
	# print("X: " + str(ret))

	errX = (target[0] - kine.ArmPos.x[8])
	errY = (target[1] - kine.ArmPos.y[8])
	errZ = (target[2] - kine.ArmPos.z[8])

	# print("Calculate Err: ")
	# print("X: " + str(errX))
	# print("Y: " + str(errY))
	# print("Z: " + str(errZ))

	# draw a simulate
	# for i in range(NUM_ANGLE):
	# 	currentAngle[i] = ArmPos.anglerad[i]

	# iter = [0,0,0,0,0,0]
	# for i in range(NUM_ANGLE):
	# 	iter[i] = (currentAngle[i] - preAngle[i]) / 10

	# ArmPos.anglerad = [preAngle[0],preAngle[1],preAngle[2],preAngle[3],preAngle[4],preAngle[5]]
	# for i in range(10):
	# 	for j in range(NUM_ANGLE):
	# 		ArmPos.anglerad[j] += iter[j]

	# 	ForwardKinematicSim()
	# 	ax.plot(ArmPos.x, ArmPos.y, ArmPos.z, label='IK')
	# 	ax.legend()

	# draw a latest
	# ForwardKinematicSim(True)
	# ax.plot(ArmPos.x, ArmPos.y, ArmPos.z, label='IK')
	# ax.legend()

	plt.show()
