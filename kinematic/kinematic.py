import math
import numpy as np

PI = 3.14
DEG_TO_RAD = PI/180
RAD_TO_DEG = 180/PI

NUM_ANGLE = 6
ARM_DOT = 9

JOINT_L1 = 130
JOINT_L2 = 75.0
JOINT_L3 = 362.61
JOINT_L4 = 75.0
JOINT_L5 = 198.0
JOINT_L5_ANGLE = 270.0 * DEG_TO_RAD
JOINT_L6 = 207.0
JOINT_L7 = 80.0
JOINT_L8 = 50.0

MAX_ANGLE = [PI, PI, PI, PI, PI, PI]
MIN_ANGLE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

class Arm_Robot:
	x = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	y = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	z = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	rx = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	ry = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	rz = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	module = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	# direct = [1,1,1,1]
	angle = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	anglerad = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

class Coordinate:
	x = 0
	y = 0
	z = 0
	rx = 0
	rx = 0
	rz = 0

ArmPos = Arm_Robot

class Kinematic:
    def __init__(self):
        print("init")

    def ArmRobotReset():
        global ArmPos
        for i in range(NUM_ANGLE):
            ArmPos.x[i] = 0
            ArmPos.y[i] = 0
            ArmPos.z[i] = 0
            ArmPos.module[i] = 0
        for j in range(ARM_DOT):
            ArmPos.angle[j] = 0
            ArmPos.anglerad[j] = 0

    def ForwardKinematic():
        global ArmPos
        ret = [0.0, 0.0, 0.0]
        # module = [0.0, 0.0, 0.0, 0.0]
        # x = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # y = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # z = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # legpos[leg].anglerad[0] = (legpos[leg].angle[0] - PULSE_BASE) * DEG_PER_PULSE * DEG_TO_RAD
        ArmPos.x[0] = 0.0
        ArmPos.y[0] = 0.0
        ArmPos.z[0] = 0.0
        # ArmPos.module[0] = 0.0

        ArmPos.x[1] = 0.0
        ArmPos.y[1] = 0.0
        ArmPos.z[1] = JOINT_L1
        # ArmPos.module[1] = 0.0

        # ArmPos.anglerad[0] = ArmPos.angle[0] * DEG_TO_RAD
        # ArmPos.module[2] = JOINT_L2
        ArmPos.x[2] = JOINT_L2 * math.cos(ArmPos.anglerad[0])
        ArmPos.y[2] = JOINT_L2 * math.sin(ArmPos.anglerad[0])
        ArmPos.z[2] = ArmPos.z[1]
        
        # ArmPos.anglerad[1] = ArmPos.angle[1] * DEG_TO_RAD
        module = JOINT_L3 * math.cos(ArmPos.anglerad[1])         
        ArmPos.x[3] = ArmPos.x[2] + module * math.cos(ArmPos.anglerad[0])
        ArmPos.y[3] = ArmPos.y[2] + module * math.sin(ArmPos.anglerad[0])
        ArmPos.z[3] = ArmPos.z[2] + JOINT_L3 * math.sin(ArmPos.anglerad[1])

        module = JOINT_L4 * math.cos(ArmPos.anglerad[1] + ArmPos.anglerad[2])
        ArmPos.x[4] = ArmPos.x[3] + module * math.cos(ArmPos.anglerad[0])
        ArmPos.y[4] = ArmPos.y[3] + module * math.sin(ArmPos.anglerad[0])
        ArmPos.z[4] = ArmPos.z[3] + JOINT_L4 * math.sin(ArmPos.anglerad[1] + ArmPos.anglerad[2])

        module = JOINT_L5 * math.cos(ArmPos.anglerad[1] + ArmPos.anglerad[2] + JOINT_L5_ANGLE)
        ArmPos.x[5] = ArmPos.x[4] + module * math.cos(ArmPos.anglerad[0])
        ArmPos.y[5] = ArmPos.y[4] + module * math.sin(ArmPos.anglerad[0])
        ArmPos.z[5] = ArmPos.z[4] + JOINT_L5 * math.sin(ArmPos.anglerad[1] + ArmPos.anglerad[2] + JOINT_L5_ANGLE)

        module = JOINT_L6 * math.cos(ArmPos.anglerad[1] + ArmPos.anglerad[2] + JOINT_L5_ANGLE)
        ArmPos.x[6] = ArmPos.x[5] + module * math.cos(ArmPos.anglerad[0])
        ArmPos.y[6] = ArmPos.y[5] + module * math.sin(ArmPos.anglerad[0])
        ArmPos.z[6] = ArmPos.z[5] + JOINT_L6 * math.sin(ArmPos.anglerad[1] + ArmPos.anglerad[2] + JOINT_L5_ANGLE)

        
        ArmPos.x[7] = ArmPos.x[6] - (JOINT_L7 + JOINT_L8) * math.sin(ArmPos.anglerad[0]) * math.cos(ArmPos.anglerad[3])
        ArmPos.y[7] = ArmPos.y[6] + (JOINT_L7 + JOINT_L8) * math.cos(ArmPos.anglerad[0]) * math.cos(ArmPos.anglerad[1] + ArmPos.anglerad[2] + ArmPos.anglerad[4] + JOINT_L5_ANGLE) - \
                    (JOINT_L7 + JOINT_L8) * math.sin(ArmPos.anglerad[0]) * math.sin(ArmPos.anglerad[3]) * math.sin(ArmPos.anglerad[1] + ArmPos.anglerad[2] + ArmPos.anglerad[4] + JOINT_L5_ANGLE)
        ArmPos.z[7] = ArmPos.z[6] + (JOINT_L7 + JOINT_L8) * math.sin(ArmPos.anglerad[3]) * math.sin(ArmPos.anglerad[0]) * math.cos(ArmPos.anglerad[1] + ArmPos.anglerad[2] + ArmPos.anglerad[4] + JOINT_L5_ANGLE) + \
                    (JOINT_L7 + JOINT_L8) * math.cos(ArmPos.anglerad[0]) * math.sin(ArmPos.anglerad[1] + ArmPos.anglerad[2] + ArmPos.anglerad[4] + JOINT_L5_ANGLE)

        # ArmPos.x[8] = ArmPos.x[7] - JOINT_L8 * math.sin(ArmPos.anglerad[0]) * math.cos(ArmPos.anglerad[3])
        # ArmPos.y[8] = ArmPos.y[7] + JOINT_L8 * math.cos(ArmPos.anglerad[0]) * math.cos(ArmPos.anglerad[1] + ArmPos.anglerad[2] + ArmPos.anglerad[4] + JOINT_L5_ANGLE) - \
        # 			JOINT_L7 * math.sin(ArmPos.anglerad[0]) * math.sin(ArmPos.anglerad[3]) * math.sin(ArmPos.anglerad[1] + ArmPos.anglerad[2] + ArmPos.anglerad[4] + JOINT_L5_ANGLE)
        # ArmPos.z[8] = ArmPos.z[7] + JOINT_L8 * math.sin(ArmPos.anglerad[3]) * math.sin(ArmPos.anglerad[0]) * math.cos(ArmPos.anglerad[1] + ArmPos.anglerad[2] + ArmPos.anglerad[4] + JOINT_L5_ANGLE) + \
        # 			JOINT_L7 * math.cos(ArmPos.anglerad[0]) * math.sin(ArmPos.anglerad[1] + ArmPos.anglerad[2] + ArmPos.anglerad[4] + JOINT_L5_ANGLE)
        
        ArmPos.x[8] = ArmPos.x[7]
        ArmPos.y[8] = ArmPos.y[7]
        ArmPos.z[8] = ArmPos.z[7]

        ret[0] = ArmPos.y[8]
        ret[1] = ArmPos.x[8]
        ret[2] = ArmPos.z[8]

        return ret

    def ForwardKinematicSim(self, full):
        A1 = [[math.cos(ArmPos.anglerad[0]), 0, math.sin(ArmPos.anglerad[0]), 0], 
            [math.sin(ArmPos.anglerad[0]), 0, - math.cos(ArmPos.anglerad[0]), 0],
            [0, 1, 0, JOINT_L1],
            [0, 0, 0, 1]]
        A2 = [[1, 0, 0, JOINT_L2],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]]

        A3 = [[math.cos(ArmPos.anglerad[1]), - math.sin(ArmPos.anglerad[1]), 0 , JOINT_L3 * math.cos(ArmPos.anglerad[1])],
            [math.sin(ArmPos.anglerad[1]), math.cos(ArmPos.anglerad[1]), 0, JOINT_L3 * math.sin(ArmPos.anglerad[1])],
            [0, 0, 1, 0],
            [0, 0, 0, 1]]
        A4 = [[math.cos(ArmPos.anglerad[2]), 0, - math.sin(ArmPos.anglerad[2]), JOINT_L4 * math.cos(ArmPos.anglerad[2])],
            [math.sin(ArmPos.anglerad[2]), 0, math.cos(ArmPos.anglerad[2]), JOINT_L4 * math.sin(ArmPos.anglerad[2])],
            [0, -1, 0, 0],
            [0, 0, 0, 1]]
        A5 = [[1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, -1 ,0],
            [0, 0, 0, 1]]
        A6 = [[math.cos(ArmPos.anglerad[3]), 0, - math.sin(ArmPos.anglerad[3]), 0],
            [math.sin(ArmPos.anglerad[3]), 0, math.cos(ArmPos.anglerad[3]), 0],
            [0, -1, 0 , JOINT_L5 + JOINT_L6],
            [0, 0, 0, 1]]
        A7 = [[math.cos(ArmPos.anglerad[4]), 0 , math.sin(ArmPos.anglerad[4]), 0],
            [math.sin(ArmPos.anglerad[4]), 0, - math.cos(ArmPos.anglerad[4]), 0],
            [0, 1, 0, 0],
            [0, 0, 0, 1]]
        A8 = [[math.cos(ArmPos.anglerad[5]), - math.sin(ArmPos.anglerad[5]), 0, 0],
            [math.sin(ArmPos.anglerad[5]), math.cos(ArmPos.anglerad[5]), 0, 0],
            [0, 0, 1, JOINT_L7 + JOINT_L8],
            [0, 0, 0, 1]]
        ArmPos.x[1] = A1[0][3]
        ArmPos.y[1] = A1[1][3]
        ArmPos.z[1] = A1[2][3]

        T2 = np.matmul(A1, A2)
        ArmPos.x[2] = T2[0][3]
        ArmPos.y[2] = T2[1][3]
        ArmPos.z[2] = T2[2][3]

        T3 = np.matmul(T2, A3)
        ArmPos.x[3] = T3[0][3]
        ArmPos.y[3] = T3[1][3]
        ArmPos.z[3] = T3[2][3]

        T4 = np.matmul(T3, A4)
        ArmPos.x[4] = T4[0][3]
        ArmPos.y[4] = T4[1][3]
        ArmPos.z[4] = T4[2][3]

        T5 = np.matmul(T4, A5)
        ArmPos.x[5] = T5[0][3]
        ArmPos.y[5] = T5[1][3]
        ArmPos.z[5] = T5[2][3]

        T6 = np.matmul(T5, A6)
        ArmPos.x[6] = T6[0][3]
        ArmPos.y[6] = T6[1][3]
        ArmPos.z[6] = T6[2][3]

        T7 = np.matmul(T6, A7)
        ArmPos.x[7] = T7[0][3]
        ArmPos.y[7] = T7[1][3]
        ArmPos.z[7] = T7[2][3]

        if full == False:
            return T7

        T8 = np.matmul(T7, A8)
        ArmPos.x[8] = T8[0][3]
        ArmPos.y[8] = T8[1][3]
        ArmPos.z[8] = T8[2][3]

        return T8

    def InverstKinematicSim(self, x, y, z, yaw, pitch, roll):
        global ArmPos
        print("target T: " + str(x) + " " + str(y) + " " + str(z))
        print("target R: " + str(yaw) + " " + str(pitch) + " " + str(roll))
        delta = 0.000001
        ddelta = delta * 2
        learnRate = 0.00001
        minErr = 0.001
        current = self.ForwardKinematicSim(self, True)
        current1 = self.ForwardKinematicSim(self, False)
        tempAngle = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        for i in range(NUM_ANGLE):
            tempAngle[i] = ArmPos.anglerad[i]

        print("Current position: " + str(current))

        rotateMat = [[math.cos(yaw) * math.cos(pitch), math.cos(yaw) * math.sin(pitch) * math.sin(roll) - math.sin(yaw) * math.cos(roll), math.cos(yaw) * math.sin(pitch) * math.cos(roll) + math.sin(yaw) * math.sin(roll)],
                    [math.sin(yaw) * math.cos(pitch), math.sin(yaw) * math.sin(pitch) * math.sin(roll) + math.cos(yaw) * math.cos(roll), math.sin(yaw) * math.sin(pitch) * math.cos(roll) - math.cos(yaw) * math.sin(roll)],
                    [-math.sin(pitch), math.cos(pitch) * math.sin(roll), math.cos(pitch) * math.cos(roll)]]

        print("Rotate matrix: " + str(rotateMat))

        pos1 = [0.0, 0.0, JOINT_L7 + JOINT_L8]
        resPos1 = [0.0, 0.0, 0.0]

        for h in range (3):
            temp = 0
            tempMul = 1
            for k in range (3):
                tempMul = pos1[k] * rotateMat[h][k]
                temp += tempMul
            resPos1[h] = temp
        
        print("resPos1: " + str(resPos1))
        tx1 = x - resPos1[0]
        ty1 = y - resPos1[1]
        tz1 = z - resPos1[2]

        print("L7 target: " + str(tx1) + " " + str(ty1) + " " + str(tz1))

        linearErr = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        rotateErr = [[0, 0, 0],
                    [0, 0, 0],
                    [0, 0, 0]]
        tempLinearErr = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        tempRotateErr = [[0, 0, 0],
                        [0, 0, 0],
                        [0, 0, 0]]
        totalErr = 0
        tempTotalErr = 0
        pre = [[0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0]]
        pre1 = [[0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0]]

        for k in range(10000):
            # current = ForwardKinematicSim()
            for i in range (NUM_ANGLE):
                currentGrad = ArmPos.anglerad[i]
                ArmPos.anglerad[i] += delta
                
                # totalErr = 0
                # tempTotalErr = 0

                for h in range (4):
                    for k in range (4):
                        pre[h][k] = current[h][k]
                        pre1[h][k] = current1[h][k]

                current = self.ForwardKinematicSim(self, True)
                current1 = self.ForwardKinematicSim(self, False)

                # calculate for linear
                linearErr[0] = 1/2 * ((x - current[0][3]) * (x - current[0][3]))
                linearErr[1] = 1/2 * ((y - current[1][3]) * (y - current[1][3]))
                linearErr[2] = 1/2 * ((z - current[2][3]) * (z - current[2][3]))

                linearErr[3] = 1/2 * ((tx1 - current1[0][3]) * (tx1 - current1[0][3]))
                linearErr[4] = 1/2 * ((ty1 - current1[1][3]) * (ty1 - current1[1][3]))
                linearErr[5] = 1/2 * ((tz1 - current1[2][3]) * (tz1 - current1[2][3]))

                # linearErr[0] = (x - current[0][3])
                # linearErr[1] = (y - current[1][3])
                # linearErr[2] = (z - current[2][3])

                totalErr = linearErr[0] + linearErr[1] + linearErr[2] + linearErr[3] + linearErr[4] + linearErr[5]

                # calculate for rotate
                # for h in range (3):
                # 	for k in range (3):
                # 		rotateErr[h][k] = 1/2 * ((rotateMat[h][k] - current[h][k]) * (rotateMat[h][k] - current[h][k]))
                # 		totalErr += rotateErr[h][k]

                # print("rotateErr: " + str(rotateErr))
                # print("rotateMat: " + str(rotateMat))
                # print("current: " + str(current))

                # calculate for linear
                tempLinearErr[0] = 1/2 * ((x - pre[0][3]) * (x - pre[0][3]))
                tempLinearErr[1] = 1/2 * ((y - pre[1][3]) * (y - pre[1][3]))
                tempLinearErr[2] = 1/2 * ((z - pre[2][3]) * (z - pre[2][3]))

                tempLinearErr[3] = 1/2 * ((tx1 - pre1[0][3]) * (tx1 - pre1[0][3]))
                tempLinearErr[4] = 1/2 * ((ty1 - pre1[1][3]) * (ty1 - pre1[1][3]))
                tempLinearErr[5] = 1/2 * ((tz1 - pre1[2][3]) * (tz1 - pre1[2][3]))

                # tempLinearErr[0] = (x - pre[0][3])
                # tempLinearErr[1] = (y - pre[1][3])
                # tempLinearErr[2] = (z - pre[2][3])

                tempTotalErr = tempLinearErr[0] + tempLinearErr[1] + tempLinearErr[2] + tempLinearErr[3] + tempLinearErr[4] + tempLinearErr[5]

                # calculate for rotate
                # for h in range (3):
                # 	for k in range (3):
                # 		tempRotateErr[h][k] = 1/2 * ((rotateMat[h][k] - pre[h][k]) * (rotateMat[h][k] - pre[h][k]))
                # 		tempTotalErr += tempRotateErr[h][k]

                # print("current pos: " + str(current))
                # print("pre pos: " + str(pre))

                deri = (totalErr - tempTotalErr) / ddelta
                # print("totalErr: " + str(totalErr))
                # print("tempTotalErr: " + str(tempTotalErr))
                # print("derivate: " + str(deri))
                # print("change: " + str(deri * learnRate))
                # if (deri * learnRate) > 2:
                # 	print("totalErr: " + str(totalErr))
                # 	print("tempTotalErr: " + str(tempTotalErr))
                # 	print("derivate: " + str(deri))
                # 	return
                
                tempAngle[i] = tempAngle[i] - (deri * learnRate)
                # if tempAngle[i] < 0:
                # 	tempAngle[i] = 0
                # if tempAngle[i] > PI:
                # 	tempAngle[i] = PI

                # print("tempAngle: " + str(tempAngle))
                ArmPos.anglerad[i] = currentGrad
                # print("tempAngle: " + str(tempAngle))

            # print("ArmPos angle: " + str(ArmPos.anglerad))
            # print("Tempos angle: " + str(tempAngle))
            for l in range(NUM_ANGLE):
                ArmPos.anglerad[l] = tempAngle[l]

            current = self.ForwardKinematicSim(self, True)
            current1 = self.ForwardKinematicSim(self, False)

            linearErr[0] = 1/2 * ((x - current[0][3]) * (x - current[0][3]))
            linearErr[1] = 1/2 * ((y - current[1][3]) * (y - current[1][3]))
            linearErr[2] = 1/2 * ((z - current[2][3]) * (z - current[2][3]))

            linearErr[3] = 1/2 * ((tx1 - current1[0][3]) * (tx1 - current1[0][3]))
            linearErr[4] = 1/2 * ((ty1 - current1[1][3]) * (ty1 - current1[1][3]))
            linearErr[5] = 1/2 * ((tz1 - current1[2][3]) * (tz1 - current1[2][3]))
            totalErr = linearErr[0] + linearErr[1] + linearErr[2] + linearErr[3] + linearErr[4] + linearErr[5]

            # linearErr[0] = (x - current[0][3])
            # linearErr[1] = (y - current[1][3])
            # linearErr[2] = (z - current[2][3])
            # totalErr = linearErr[0] + linearErr[1] + linearErr[2]
            # print("totalErr: " + str(totalErr))

            # for h in range (3):
            # 	for k in range (3):
            # 		rotateErr[h][k] = 1/2 * ((rotateMat[h][k] - current[h][k]) * (rotateMat[h][k] - current[h][k]))
            # 		totalErr += rotateErr[h][k]

            # print("total err:" + str(totalErr))
            if totalErr <= minErr:
                print("totalErr: " + str(totalErr))
                break
		

    def InverstKinematicSimNew(self, x, y, z, roll, pitch, yaw):
        global ArmPos
        print("target T: " + str(x) + " " + str(y) + " " + str(z))
        print("target R: " + str(roll) + " " + str(pitch) + " " + str(yaw))
        delta = 0.000001
        ddelta = delta * 2
        learnRate = 0.00001
        # learnRate1 = 0.0001
        minErr = 0.001
        current = self.ForwardKinematicSim(self, True)
        tempAngle = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        for i in range(NUM_ANGLE):
            tempAngle[i] = ArmPos.anglerad[i]

        print("Current position: " + str(current))

        rotateMat = [[math.cos(roll) * math.cos(pitch), math.cos(roll) * math.sin(pitch) * math.sin(yaw) - math.sin(roll) * math.cos(yaw), math.cos(roll) * math.sin(pitch) * math.cos(yaw) + math.sin(roll) * math.sin(yaw)],
                    [math.sin(roll) * math.cos(pitch), math.sin(roll) * math.sin(pitch) * math.sin(yaw) + math.cos(roll) * math.cos(yaw), math.sin(roll) * math.sin(pitch) * math.cos(yaw) - math.cos(roll) * math.sin(yaw)],
                    [-math.sin(pitch), math.cos(pitch) * math.sin(yaw), math.cos(pitch) * math.cos(yaw)]]

        print("Rotate matrix: " + str(rotateMat))

        linearErr = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        rotateErr = [[0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0]]
        tempLinearErr = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        tempRotateErr = [[0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0]]
        totalErr = 0.0
        tempTotalErr = 0.0
        pre = [[0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0]]

        for k in range(1000):
            # current = ForwardKinematicSim()
            for i in range (NUM_ANGLE):
                currentGrad = ArmPos.anglerad[i]
                ArmPos.anglerad[i] += delta
                
                # totalErr = 0
                # tempTotalErr = 0

                for h in range (4):
                    for k in range (4):
                        pre[h][k] = current[h][k]

                current = self.ForwardKinematicSim(self, True)

                # calculate for linear
                linearErr[0] = 1/2 * ((x - current[0][3]) * (x - current[0][3]))
                linearErr[1] = 1/2 * ((y - current[1][3]) * (y - current[1][3]))
                linearErr[2] = 1/2 * ((z - current[2][3]) * (z - current[2][3]))

                # linearErr[0] = (x - current[0][3])
                # linearErr[1] = (y - current[1][3])
                # linearErr[2] = (z - current[2][3])

                totalErr = linearErr[0] + linearErr[1] + linearErr[2]

                # calculate for rotate
                for h in range (3):
                    for k in range (3):
                        rotateErr[h][k] = 1/2 * ((rotateMat[h][k] - current[h][k]) * (rotateMat[h][k] - current[h][k]))
                        totalErr += rotateErr[h][k]

                # print("rotateErr: " + str(rotateErr))
                # print("rotateMat: " + str(rotateMat))
                # print("current: " + str(current))

                # calculate for linear
                tempLinearErr[0] = 1/2 * ((x - pre[0][3]) * (x - pre[0][3]))
                tempLinearErr[1] = 1/2 * ((y - pre[1][3]) * (y - pre[1][3]))
                tempLinearErr[2] = 1/2 * ((z - pre[2][3]) * (z - pre[2][3]))

                # tempLinearErr[0] = (x - pre[0][3])
                # tempLinearErr[1] = (y - pre[1][3])
                # tempLinearErr[2] = (z - pre[2][3])

                tempTotalErr = tempLinearErr[0] + tempLinearErr[1] + tempLinearErr[2]

                # calculate for rotate
                for h in range (3):
                    for k in range (3):
                        tempRotateErr[h][k] = 1/2 * ((rotateMat[h][k] - pre[h][k]) * (rotateMat[h][k] - pre[h][k]))
                        tempTotalErr += tempRotateErr[h][k]

                # print("current pos: " + str(current))
                # print("pre pos: " + str(pre))

                deri = (totalErr - tempTotalErr) / ddelta
                change = deri * learnRate
                # print("totalErr: " + str(totalErr))
                # print("tempTotalErr: " + str(tempTotalErr))
                # print("derivate: " + str(deri))
                # print("change: " + str(change))
                # if (deri * learnRate) > 2.0:
                # 	print("totalErr: " + str(totalErr))
                # 	print("tempTotalErr: " + str(tempTotalErr))
                # 	print("derivate: " + str(deri))
                # 	return
                
                tempAngle[i] = tempAngle[i] - change
                # if tempAngle[i] < 0:
                # 	tempAngle[i] = 0
                # if tempAngle[i] > PI:
                # 	tempAngle[i] = PI

                # print("tempAngle: " + str(tempAngle))
                ArmPos.anglerad[i] = currentGrad
                # print("tempAngle: " + str(tempAngle))

            # print("ArmPos angle: " + str(ArmPos.anglerad))
            # print("Tempos angle: " + str(tempAngle))
            for l in range(NUM_ANGLE):
                ArmPos.anglerad[l] = tempAngle[l]

            current = self.ForwardKinematicSim(self, True)

            linearErr[0] = 1/2 * ((x - current[0][3]) * (x - current[0][3]))
            linearErr[1] = 1/2 * ((y - current[1][3]) * (y - current[1][3]))
            linearErr[2] = 1/2 * ((z - current[2][3]) * (z - current[2][3]))

            totalErr = linearErr[0] + linearErr[1] + linearErr[2] 
            # linearErr[0] = (x - current[0][3])
            # linearErr[1] = (y - current[1][3])
            # linearErr[2] = (z - current[2][3])
            # totalErr = linearErr[0] + linearErr[1] + linearErr[2]
            # print("totalErr: " + str(totalErr))

            for h in range (3):
                for k in range (3):
                    rotateErr[h][k] = 1/2 * ((rotateMat[h][k] - current[h][k]) * (rotateMat[h][k] - current[h][k]))
                    totalErr += rotateErr[h][k]

            print("total err:" + str(totalErr))
            if totalErr <= minErr:
                break
