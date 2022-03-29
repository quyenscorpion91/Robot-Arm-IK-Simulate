import math
import numpy as np
import sys

PI = math.pi
DEG_TO_RAD = PI/180
RAD_TO_DEG = 180/PI

NUM_ANGLE = 6
NUM_ANGLE_0_3 = 3
NUM_ANGLE_3_6 = 3
NUM_3_6_BASE = 3
NUM_ANGLE_4_5 = 2
NUM_4_5_BASE = 3
NUM_4_5_LAST = 5
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

JOINT_EZ = 10
JOINT_EX = 10

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

        if full == False:
            return T6

        T7 = np.matmul(T6, A7)
        ArmPos.x[7] = T7[0][3]
        ArmPos.y[7] = T7[1][3]
        ArmPos.z[7] = T7[2][3]

        T8 = np.matmul(T7, A8)
        ArmPos.x[8] = T8[0][3]
        ArmPos.y[8] = T8[1][3]
        ArmPos.z[8] = T8[2][3]

        return T8

    def InverseKinematicSim(self, x, y, z, yaw, pitch, roll):
        global ArmPos
        print("target T: " + str(x) + " " + str(y) + " " + str(z))
        print("target R: " + str(yaw) + " " + str(pitch) + " " + str(roll))
        deltaLinear = 0.0001
        deltaRot = 0.001
        ddeltaLinear = deltaLinear * 2
        ddeltaRot = deltaRot * 2
        learnRateLinear = 0.00001
        learnRateLinear1 = 0.0001
        learnRateRot = 0.1
        minErrLinear = 0.001
        minErrRot = 0.0001
        currentPos06 = self.ForwardKinematicSim(self, True)
        currentPos03 = self.ForwardKinematicSim(self, False)
        tempAngle = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        for i in range(NUM_ANGLE):
            tempAngle[i] = ArmPos.anglerad[i]

        print("Current position: " + str(currentPos06))

        rotateMat = [[math.cos(yaw) * math.cos(pitch), math.cos(yaw) * math.sin(pitch) * math.sin(roll) - math.sin(yaw) * math.cos(roll), math.cos(yaw) * math.sin(pitch) * math.cos(roll) + math.sin(yaw) * math.sin(roll)],
                    [math.sin(yaw) * math.cos(pitch), math.sin(yaw) * math.sin(pitch) * math.sin(roll) + math.cos(yaw) * math.cos(roll), math.sin(yaw) * math.sin(pitch) * math.cos(roll) - math.cos(yaw) * math.sin(roll)],
                    [-math.sin(pitch), math.cos(pitch) * math.sin(roll), math.cos(pitch) * math.cos(roll)]]

        print("Rotate matrix: " + str(rotateMat))
        desireRot06 = self.GetRotateAngleFromMatrix(self, rotateMat)
        print("desireRot06: " + str(desireRot06))
        currentRot06 = self.GetRotateAngleFromMatrix(self, currentPos06)
        print("currentRot06: " + str(currentRot06))

        endEffector = [0.0, 0.0, JOINT_L7 + JOINT_L8]

        endEffectorOrient = np.matmul(rotateMat, endEffector)
        print("endEffectorOrient: " + str(endEffectorOrient))
        desiredPos03 = [0.0, 0.0, 0.0]
        desiredPos03[0] = x - endEffectorOrient[0]
        desiredPos03[1] = y - endEffectorOrient[1]
        desiredPos03[2] = z - endEffectorOrient[2]
        print("desiredPos03: " + str(desiredPos03))

        prePos03 = [[0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0]]
        prePos06 = [[0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0]]
        preRot06 = [0.0, 0.0, 0.0]

        currentLinearErr = [0.0, 0.0, 0.0]
        currentRotateErr = [0.0, 0.0, 0.0]

        preLinearErr = [0.0, 0.0, 0.0]
        preRotateErr = [0.0, 0.0, 0.0]

        currentLinearTotalErr = 0.0
        preLinearTotalErr = 0.0

        currentRotTotalErr = 0.0
        preRotTotalErr = 0.0

        # 0_3 position calculate
        for k in range(1000):
            # update for position (0_3)
            for i in range (NUM_ANGLE_0_3):
                currentGrad = ArmPos.anglerad[i]
                ArmPos.anglerad[i] += deltaLinear

                for m in range (4):
                    for n in range (4):
                        prePos03[m][n] = currentPos03[m][n]

                currentPos03 = self.ForwardKinematicSim(self, False)

                # calculate for current linear
                currentLinearErr[0] = ((desiredPos03[0] - currentPos03[0][3]) * (desiredPos03[0] - currentPos03[0][3]))
                currentLinearErr[1] = ((desiredPos03[1] - currentPos03[1][3]) * (desiredPos03[1] - currentPos03[1][3]))
                currentLinearErr[2] = ((desiredPos03[2] - currentPos03[2][3]) * (desiredPos03[2] - currentPos03[2][3]))

                currentLinearTotalErr = (currentLinearErr[0] + currentLinearErr[1] + currentLinearErr[2]) / 3.0

                # calculate for previous linear
                preLinearErr[0] = ((desiredPos03[0] - prePos03[0][3]) * (desiredPos03[0] - prePos03[0][3]))
                preLinearErr[1] = ((desiredPos03[1] - prePos03[1][3]) * (desiredPos03[1] - prePos03[1][3]))
                preLinearErr[2] = ((desiredPos03[2] - prePos03[2][3]) * (desiredPos03[2] - prePos03[2][3]))

                preLinearTotalErr = (preLinearErr[0] + preLinearErr[1] + preLinearErr[2]) / 3.0


                deri = (currentLinearTotalErr - preLinearTotalErr) / ddeltaLinear
                
                tempAngle[i] = tempAngle[i] - (deri * learnRateLinear)
                # if(tempAngle[i] > MAX_ANGLE[i]):
                #     tempAngle[i] = MAX_ANGLE[i]
                # if(tempAngle[i] < MIN_ANGLE[i]):
                #     tempAngle[i] = MIN_ANGLE[i]

                ArmPos.anglerad[i] = currentGrad

            # update angle for arm
            for l in range(NUM_ANGLE_0_3):
                ArmPos.anglerad[l] = tempAngle[l]

            currentPos03 = self.ForwardKinematicSim(self, False)

            # calculate for current linear error
            currentLinearErr[0] = ((desiredPos03[0] - currentPos03[0][3]) * (desiredPos03[0] - currentPos03[0][3]))
            currentLinearErr[1] = ((desiredPos03[1] - currentPos03[1][3]) * (desiredPos03[1] - currentPos03[1][3]))
            currentLinearErr[2] = ((desiredPos03[2] - currentPos03[2][3]) * (desiredPos03[2] - currentPos03[2][3]))

            currentLinearTotalErr = (currentLinearErr[0] + currentLinearErr[1] + currentLinearErr[2]) / 3.0
            # print("currentLinearTotalErr: " + str(currentLinearTotalErr))
            if currentLinearTotalErr <= minErrLinear:
                print("currentLinearTotalErr: " + str(currentLinearTotalErr))
                print("loop count: " + str(k))
                break

        # 4_5 positin calculate
        for k in range(1000):
            # update for position (0_3)
            for i in range (NUM_4_5_BASE, NUM_4_5_LAST):
                currentGrad = ArmPos.anglerad[i]
                ArmPos.anglerad[i] += deltaLinear

                for m in range (4):
                    for n in range (4):
                        prePos06[m][n] = currentPos06[m][n]

                currentPos06 = self.ForwardKinematicSim(self, True)

                # calculate for current linear
                currentLinearErr[0] = ((x - currentPos06[0][3]) * (x - currentPos06[0][3]))
                currentLinearErr[1] = ((y - currentPos06[1][3]) * (y - currentPos06[1][3]))
                currentLinearErr[2] = ((z - currentPos06[2][3]) * (z - currentPos06[2][3]))

                currentLinearTotalErr = (currentLinearErr[0] + currentLinearErr[1] + currentLinearErr[2]) / 3.0

                # calculate for previous linear
                preLinearErr[0] = ((x - prePos06[0][3]) * (x - prePos06[0][3]))
                preLinearErr[1] = ((y - prePos06[1][3]) * (y - prePos06[1][3]))
                preLinearErr[2] = ((z - prePos06[2][3]) * (z - prePos06[2][3]))

                preLinearTotalErr = (preLinearErr[0] + preLinearErr[1] + preLinearErr[2]) / 3.0

                deri = (currentLinearTotalErr - preLinearTotalErr) / ddeltaLinear
                
                tempAngle[i] = tempAngle[i] - (deri * learnRateLinear1)
                # if(tempAngle[i] > MAX_ANGLE[i]):
                #     tempAngle[i] = MAX_ANGLE[i]
                # if(tempAngle[i] < MIN_ANGLE[i]):
                #     tempAngle[i] = MIN_ANGLE[i]

                ArmPos.anglerad[i] = currentGrad

            # update angle for arm
            for l in range(NUM_4_5_BASE, NUM_4_5_LAST):
                ArmPos.anglerad[l] = tempAngle[l]

            currentPos06 = self.ForwardKinematicSim(self, True)

            # calculate for current linear
            currentLinearErr[0] = ((x - currentPos06[0][3]) * (x - currentPos06[0][3]))
            currentLinearErr[1] = ((y - currentPos06[1][3]) * (y - currentPos06[1][3]))
            currentLinearErr[2] = ((z - currentPos06[2][3]) * (z - currentPos06[2][3]))

            currentLinearTotalErr = (currentLinearErr[0] + currentLinearErr[1] + currentLinearErr[2]) / 3.0
            # print("currentLinearTotalErr: " + str(currentLinearTotalErr))
            if currentLinearTotalErr <= minErrLinear:
                print("currentLinearTotalErr 1: " + str(currentLinearTotalErr))
                print("loop count: " + str(k))
                break

        # 0_6 orientation calculate
        for k in range(1000):
            # update for end-effector orientation
            currentGrad = ArmPos.anglerad[5]
            ArmPos.anglerad[5] += deltaRot

            for m in range (3):
                preRot06[m] = currentRot06[m]

            currentPos06 = self.ForwardKinematicSim(self, True)
            currentRot06 = self.GetRotateAngleFromMatrix(self, currentPos06)

            # calculate current orientation
            currentRotateErr[0] = ((desireRot06[0] - currentRot06[0]) * (desireRot06[0] - currentRot06[0]))
            currentRotateErr[1] = ((desireRot06[1] - currentRot06[1]) * (desireRot06[1] - currentRot06[1]))
            currentRotateErr[2] = ((desireRot06[2] - currentRot06[2]) * (desireRot06[2] - currentRot06[2]))

            currentRotTotalErr = (currentRotateErr[0] + currentRotateErr[1] + currentRotateErr[2]) / 3.0

            # calculate previous orientation
            preRotateErr[0] = ((desireRot06[0] - preRot06[0]) * (desireRot06[0] - preRot06[0]))
            preRotateErr[1] = ((desireRot06[1] - preRot06[1]) * (desireRot06[1] - preRot06[1]))
            preRotateErr[2] = ((desireRot06[2] - preRot06[2]) * (desireRot06[2] - preRot06[2]))

            preRotTotalErr = (preRotateErr[0] + preRotateErr[1] + preRotateErr[2]) / 3.0

            deri = (currentRotTotalErr - preRotTotalErr) / ddeltaRot
            # print("===================================")
            # print("change: " + str(deri * learnRateRot))
            
            tempAngle[5] = tempAngle[5] - (deri * learnRateRot)
            # if(tempAngle[t] > MAX_ANGLE[t]):
            #     tempAngle[t] = MAX_ANGLE[t]
            # if(tempAngle[t] < MIN_ANGLE[t]):
            #     tempAngle[t] = MIN_ANGLE[t]

            # update angle for arm
            ArmPos.anglerad[5] = tempAngle[5]

            # calculate for current orientation
            currentPos06 = self.ForwardKinematicSim(self, True)
            currentRot06 = self.GetRotateAngleFromMatrix(self, currentPos06)

            # calculate current orientation error
            currentRotateErr[0] = ((desireRot06[0] - currentRot06[0]) * (desireRot06[0] - currentRot06[0]))
            currentRotateErr[1] = ((desireRot06[1] - currentRot06[1]) * (desireRot06[1] - currentRot06[1]))
            currentRotateErr[2] = ((desireRot06[2] - currentRot06[2]) * (desireRot06[2] - currentRot06[2]))

            currentRotTotalErr = (currentRotateErr[0] + currentRotateErr[1] + currentRotateErr[2]) / 3.0
            
            # print("currentRotTotalErr: " + str(currentRotTotalErr))
            if currentRotTotalErr <= minErrRot:
                print("currentRotTotalErr: " + str(currentRotTotalErr))
                print("loop count: " + str(k))
                break

    def Division(n , d):
        # return n / d if d else sys.float_info.max
        # print("PI: " + str(PI))
        # print("n: " + str(n))
        # print("d: " + str(d))
        if (d == 0):
            if (n == 0):
                return 0
            else:
                return sys.float_info.max

        return n / d
    
    def GetRotateAngleFromMatrix(self, rotate):
        angleRet = [0.0, 0.0, 0.0]

        # angleRet[0] = math.atan(self.Division(rotate[1][0], rotate[0][0])) # yaw
        # angleRet[1] = math.atan(self.Division(-rotate[2][0], (math.sqrt(rotate[2][1] * rotate[2][1] + rotate[2][2] * rotate[2][2])))) #pitch
        # angleRet[2] = math.atan(self.Division(rotate[2][1], rotate[2][2])) # roll

        angleRet[0] = math.atan2(rotate[1][0], rotate[0][0]) # yaw
        angleRet[1] = math.atan2(-rotate[2][0], (math.sqrt((rotate[2][1] * rotate[2][1]) + (rotate[2][2] * rotate[2][2])))) # pitch
        angleRet[2] = math.atan2(rotate[2][1], rotate[2][2]) # roll

        return angleRet

