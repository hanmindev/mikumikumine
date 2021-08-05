import os
from tkinter import Tk, filedialog
import binascii
import struct
import math
import sys

# minecraft ticks
tickno = 0

# source_file = filedialog.askopenfile().name
# output_directory = filedialog.askdirectory()


source_file = 'C:/Users/Hanmin/Documents/imas/files/goodies/songs/alive1.vmd'
output_directory = 'C:/Users/Hanmin/AppData/Roaming/.minecraft/saves/mikumikudance/datapacks/mmd/data/mmd/functions/alive1'
# 'C:/Users/Hanmin/Documents/imas/files/goodies/songs/alive2.vmd'
# 'C:/Users/Hanmin/AppData/Roaming/.minecraft/saves/mikumikudance/datapacks/mmd/data/mmd/functions/alive2'
if source_file is None or output_directory == '':
    print("This file or directory doesn't seem to exist.")
    sys.exit()

lastFolderName = output_directory[output_directory.find("datapacks/mmd/data/mmd/functions") + 33:len(output_directory)]
if output_directory.find("datapacks/mmd/data/mmd/functions") == -1:
    print("This doesn't seem like a minecraft directory... Are you sure you are in the right place?")
    sys.exit()


def q_conjugate(q):
    # returns the conjugate of a quaternion
    return [q[0], -q[1], -q[2], -q[3]]


def q_inverse(q):
    # returns the inverse of a quaternion
    modp2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]
    return [q_conjugate(q)[0] / modp2, q_conjugate(q)[1] / modp2, q_conjugate(q)[2] / modp2, q_conjugate(q)[3] / modp2]


def q_multiplication(q1, q2):
    # for some reason its not commutative lmao
    x = q1[3] * q2[0] + q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1]
    y = q1[3] * q2[1] - q1[0] * q2[2] + q1[1] * q2[3] + q1[2] * q2[0]
    z = q1[3] * q2[2] + q1[0] * q2[1] - q1[1] * q2[0] + q1[2] * q2[3]
    w = q1[3] * q2[3] - q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2]
    return [x, y, z, w]


def q_decom(q, initvector=(1, 0, 0)):
    # decomposes(?) the quaternion into a vector by rotating initvector by the quaternion
    a = q[3]
    b = q[0]
    c = q[1]
    d = q[2]

    r11 = a * a + b * b - c * c - d * d
    r21 = 2 * b * c + 2 * a * d
    r31 = 2 * b * d - 2 * a * c
    r12 = 2 * b * c - 2 * a * d
    r22 = a * a - b * b + c * c - d * d
    r32 = 2 * c * d + 2 * a * b
    r13 = 2 * b * d + 2 * a * c
    r23 = 2 * c * d - 2 * a * b
    r33 = a * a - b * b - c * c + d * d

    return [initvector[0] * r11 + initvector[1] * r12 + initvector[2] * r13,
            initvector[0] * r21 + initvector[1] * r22 + initvector[2] * r23,
            initvector[0] * r31 + initvector[1] * r32 + initvector[2] * r33]


def q_parentchild(parent, child):
    # returns the resultant quaternion after transforming the child relative to parent quaternion
    # am I writing in math language or am I just spitting out gibberish that sounds mathematical? I hate no idea
    # frick I did this wrong it's literally just multiplication google huh
    # return qMultiplication(qMultiplication(parent,child),qInverse(parent))
    # oh its how you do it to a vector gotcha lol
    return q_multiplication(parent, child)


def normalizeVector(vector):
    # normalizes vector
    modulus = vector_modulus(vector)
    return [vector[0] / modulus, vector[1] / modulus, vector[2] / modulus]


def vector_modulus(vector):
    # return modulus
    return math.sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2])


def qToE(q, order='xyz'):
    # quaternion to euler angles XYZ
    angles = [0, 0, 0]
    qx = q[0]
    qy = q[1]
    qz = q[2]
    qw = q[3]

    if order == 'xyz':
        # pitch (y-axis rotation)
        sinp = 2 * (qx * qz + qw * qy)
        angles[1] = math.asin(max(-1, min(sinp, 1)))

        # roll (x-axis rotation)
        sinr_cosp = 2 * (qy * qz - qw * qx)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        angles[0] = math.atan2(-sinr_cosp, cosr_cosp)

        # yaw (z-axis rotation)
        siny_cosp = 2 * (qx * qy - qw * qz)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        angles[2] = math.atan2(-siny_cosp, cosy_cosp)
    elif order == 'yxz':
        sinp = 2 * (qy * qz - qw * qx)
        angles[0] = math.asin(-1 * max(-1, min(sinp, 1)))

        sinr_cosp = 2 * (qx * qz + qw * qy)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        angles[1] = math.atan2(sinr_cosp, cosr_cosp)

        siny_cosp = 2 * (qx * qy + qw * qz)
        cosy_cosp = 1 - 2 * (qx * qx + qz * qz)
        angles[2] = math.atan2(siny_cosp, cosy_cosp)
    elif order == 'zxy':
        sinp = 2 * (qy * qz + qw * qx)
        angles[0] = math.asin(max(-1, min(sinp, 1)))

        sinr_cosp = 2 * (qx * qz - qw * qy)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        angles[1] = math.atan2(-sinr_cosp, cosr_cosp)

        siny_cosp = 2 * (qx * qy - qw * qz)
        cosy_cosp = 1 - 2 * (qx * qx + qz * qz)
        angles[2] = math.atan2(-siny_cosp, cosy_cosp)
    elif order == 'zyx':
        sinp = 2 * (qw * qy - qz * qx)
        angles[1] = math.asin(max(-1, min(sinp, 1)))

        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        angles[0] = math.atan2(sinr_cosp, cosr_cosp)

        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        angles[2] = math.atan2(siny_cosp, cosy_cosp)
    elif order == 'yzx':
        sinp = 2 * (qx * qy + qw * qz)
        angles[2] = math.asin(max(-1, min(sinp, 1)))

        sinr_cosp = 2 * (qy * qz - qw * qx)
        cosr_cosp = 1 - 2 * (qx * qx + qz * qz)
        angles[0] = math.atan2(-sinr_cosp, cosr_cosp)

        siny_cosp = 2 * (qx * qz - qw * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        angles[1] = math.atan2(-siny_cosp, cosy_cosp)
    elif order == 'xzy':
        sinp = 2 * (qx * qy - qw * qz)
        angles[2] = math.asin(-1 * max(-1, min(sinp, 1)))

        sinr_cosp = 2 * (qy * qz + qw * qx)
        cosr_cosp = 1 - 2 * (qx * qx + qz * qz)
        angles[0] = math.atan2(sinr_cosp, cosr_cosp)

        siny_cosp = 2 * (qx * qz + qw * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        angles[1] = math.atan2(siny_cosp, cosy_cosp)

    for j in range(len(angles)):
        angles[j] *= (180 / math.pi)

    return angles


def dotProd(s1, s2):
    output = []
    for j in range(len(s1)):
        output.append(s1[j] * s2[j])
    return output


def stringify(floatstring):
    # turns array of floats into a string such as "[0.0,0.0,0.0]" for processing in minecraft
    temp_string = "["
    for j in range(len(floatstring)):
        temp_string += str(f"{floatstring[j]:.9f}")
        if j < len(floatstring) - 1:
            temp_string += ","
        else:
            temp_string += "]"
    return temp_string


class Frame:
    def __init__(self):
        self.headQ = [0, 0, 0, 0]
        self.rightShoulderQ = [0, 0, 0, 0]
        self.rightArmQ = [0, 0, 0, 0]
        self.rightWristQ = [0, 0, 0, 0]
        self.rightElbowQ = [0, 0, 0, 0]
        self.leftShoulderQ = [0, 0, 0, 0]
        self.leftArmQ = [0, 0, 0, 0]
        self.leftWristQ = [0, 0, 0, 0]
        self.leftElbowQ = [0, 0, 0, 0]
        self.rightLegQ = [0, 0, 0, 0]
        self.leftLegQ = [0, 0, 0, 0]
        self.rightKneeQ = [0, 0, 0, 0]
        self.leftKneeQ = [0, 0, 0, 0]
        self.rightAnkleQ = [0, 0, 0, 0]
        self.leftAnkleQ = [0, 0, 0, 0]
        self.rightToeQ = [0, 0, 0, 0]
        self.leftToeQ = [0, 0, 0, 0]
        self.bodyQ = [0, 0, 0, 0]
        self.pos = [0, 0, 0]
        self.grooveQ = [0, 0, 0, 0]
        self.groovePos = [0, 0, 0]
        self.waistQ = [0, 0, 0, 0]


# your MMD model needs these following bones.
translation = {
    b'835a8393835e815b00000000000000': "center",
    b'834f838b815b837500000000000000': "groove",
    b'8fe394bc9067000000000000000000': "upper body",
    b'8fe394bc9067320000000000000000': "upper body2",
    b'8db68ca80000000000000000000000': "shoulder L",
    b'8db698720000000000000000000000': "arm L",
    b'8db682d082b6000000000000000000': "elbow L",
    b'8db68ee88ef1000000000000000000': "wrist L",
    b'89458ca80000000000000000000000': "shoulder R",
    b'894598720000000000000000000000': "arm R",
    b'894582d082b6000000000000000000': "elbow R",
    b'89458ee88ef1000000000000000000': "wrist R",
    b'8ef100000000000000000000000000': "neck",
    b'8d9800000000000000000000000000': "waist",
    b'8db691ab0000000000000000000000': "leg L",
    b'8db682d082b4000000000000000000': "knee L",
    b'8db691ab8ef1000000000000000000': "ankle L",
    b'8db682c282dc90e600000000000000': "toe L",
    b'894591ab0000000000000000000000': "leg R",
    b'894582d082b4000000000000000000': "knee R",
    b'894591ab8ef1000000000000000000': "ankle R",
    b'894582c282dc90e600000000000000': "toe R",
    b'91808dec9286905300000000000000': "main",
    b'915382c482cc906500000000000000': "mother",
    b'93aa00000000000000000000000000': "head",
    b'000000000000000000000000000000': "unknown"
}

with open(source_file, "rb") as f:
    print(f.read(30))
    print(f.read(20))
    length = f.read(4)
    print(length)
    keyframeCount = int.from_bytes(length, "little")
    print(keyframeCount)

    bonepos = [0, 0, 0]
    bonerot = [0, 0, 0, 0]

    # only use the skipth frame
    skip = 3
    partcount = 0
    currentframe = 0
    current = Frame()
    mcFrameCount = int(keyframeCount / 59 / skip)
    binaryLength = math.ceil(math.log2(mcFrameCount))
    binaryTreeChildren = []
    lastPos = [0, 0, 0]
    positionShift = [0, 0, 0]
    for frame in list(range(keyframeCount)):
        name = binascii.b2a_hex(f.read(15))
        if name == b'000000000000000000000000000000':
            break
        try:
            translation[name]
        except:
            f.read(111 - 15)
            continue

        framecount = int.from_bytes(f.read(4), "little")
        if framecount == 0:
            partcount += 1
        elif framecount % skip != 0:
            f.read(111 - 19)
            continue

        if framecount != currentframe:
            currentframe = framecount

            # output mode
            if 1:

                rightShoulderQ_i = current.rightShoulderQ
                rightArmQ_i = current.rightArmQ
                rightElbowQ_i = current.rightElbowQ
                rightWristQ_i = current.rightWristQ

                leftShoulderQ_i = current.leftShoulderQ
                leftArmQ_i = current.leftArmQ
                leftElbowQ_i = current.leftElbowQ
                leftWristQ_i = current.leftWristQ

                rightLegQ_i = current.rightLegQ
                rightKneeQ_i = current.rightKneeQ
                rightAnkleQ_i = current.rightAnkleQ
                rightToeQ_i = current.rightToeQ

                leftLegQ_i = current.leftLegQ
                leftKneeQ_i = current.leftKneeQ
                leftAnkleQ_i = current.leftAnkleQ
                leftToeQ_i = current.leftToeQ

                position_i = current.pos
                groove_i = current.grooveQ
                groovePos = current.groovePos
                upperBody_i = current.upperBodyQ
                upperBody2_i = current.upperBody2Q
                waist_i = current.waistQ

                neck_i = current.neckQ
                head_i = current.headQ

                # parented joints
                upperBody_p = q_parentchild(groove_i, upperBody_i)
                upperBody2_p = q_parentchild(upperBody_p, upperBody2_i)

                neck_p = q_parentchild(upperBody2_p, neck_i)
                head_p = q_parentchild(neck_p, head_i)

                waist_p = q_parentchild(groove_i, waist_i)

                rightShoulderQ_p = q_parentchild(upperBody2_p, rightShoulderQ_i)
                rightArmQ_p = q_parentchild(rightShoulderQ_p, rightArmQ_i)
                rightElbowQ_p = q_parentchild(rightArmQ_p, rightElbowQ_i)
                rightWristQ_p = q_parentchild(rightElbowQ_p, rightWristQ_i)

                leftShoulderQ_p = q_parentchild(upperBody2_p, leftShoulderQ_i)
                leftArmQ_p = q_parentchild(leftShoulderQ_p, leftArmQ_i)
                leftElbowQ_p = q_parentchild(leftArmQ_p, leftElbowQ_i)
                leftWristQ_p = q_parentchild(leftElbowQ_p, leftWristQ_i)

                rightLegQ_p = q_parentchild(waist_p, rightLegQ_i)
                rightKneeQ_p = q_parentchild(rightLegQ_p, rightKneeQ_i)
                rightAnkleQ_p = q_parentchild(rightKneeQ_p, rightAnkleQ_i)
                rightToeQ_p = q_parentchild(rightAnkleQ_p, rightToeQ_i)

                leftLegQ_p = q_parentchild(waist_p, leftLegQ_i)
                leftKneeQ_p = q_parentchild(leftLegQ_p, leftKneeQ_i)
                leftAnkleQ_p = q_parentchild(leftKneeQ_p, leftAnkleQ_i)
                leftToeQ_p = q_parentchild(leftAnkleQ_p, leftToeQ_i)


                # export to minecraft
                def formatVector(v, fx=1, fy=2, fz=3):
                    # fx,fy,fz determines what component of the vector will go to which slot. if negative, flip
                    # none is 0, x is 1, y is 2, z is 3
                    x, y, z = v[0], v[1], v[2]
                    output = []
                    for j in [fx, fy, fz]:
                        if j < 0:
                            copysign = -1
                        else:
                            copysign = 1
                        if j == 0:
                            output.append(0)
                        elif abs(j) == 1:
                            output.append(x * copysign)
                        elif abs(j) == 2:
                            output.append(y * copysign)
                        elif abs(j) == 3:
                            output.append(z * copysign)
                    return "^" + str('{:f}'.format(output[0])) + " ^" + str('{:f}'.format(output[1])) + " ^" + str(
                        '{:f}'.format(output[2]))


                currentTick = int(currentframe / skip - 1)

                # file binary name
                fileNumber = bin(currentTick)[2:].zfill(binaryLength + 1)
                binaryTreeChildren.append(fileNumber)

                completeName = os.path.join(output_directory, str(fileNumber) + ".mcfunction")
                open(completeName, 'w').close()
                g = open(completeName, "a")
                g.write("# frame " + str(currentTick) + "\n")
                g.write("tp @s[tag=r_shoulder_p] " + formatVector(
                    dotProd(q_decom(rightShoulderQ_p, [1, 0, 0]), [1, -1, -1])) + "\n")
                g.write(
                    "tp @s[tag=r_arm_p] " + formatVector(dotProd(q_decom(rightArmQ_p, [1, 0, 0]), [1, -1, -1])) + "\n")
                g.write("tp @s[tag=r_elbow_p] " + formatVector(
                    dotProd(q_decom(rightElbowQ_p, [1, 0, 0]), [1, -1, -1])) + "\n")
                g.write("tp @s[tag=r_wrist_p] " + formatVector(
                    dotProd(q_decom(rightWristQ_p, [1, 0, 0]), [1, -1, -1])) + "\n")

                g.write("tp @s[tag=l_shoulder_p] " + formatVector(
                    dotProd(q_decom(leftShoulderQ_p, [1, 0, 0]), [-1, 1, 1])) + "\n")
                g.write(
                    "tp @s[tag=l_arm_p] " + formatVector(dotProd(q_decom(leftArmQ_p, [1, 0, 0]), [-1, 1, 1])) + "\n")
                g.write(
                    "tp @s[tag=l_elbow_p] " + formatVector(
                        dotProd(q_decom(leftElbowQ_p, [1, 0, 0]), [-1, 1, 1])) + "\n")
                g.write(
                    "tp @s[tag=l_wrist_p] " + formatVector(
                        dotProd(q_decom(leftWristQ_p, [1, 0, 0]), [-1, 1, 1])) + "\n")

                g.write(
                    "tp @s[tag=r_leg_p] " + formatVector(dotProd(q_decom(rightLegQ_p, [0, -1, 0]), [-1, 1, 1])) + "\n")
                g.write(
                    "tp @s[tag=r_knee_p] " + formatVector(
                        dotProd(q_decom(rightKneeQ_p, [0, -1, 0]), [-1, 1, 1])) + "\n")
                g.write("tp @s[tag=r_ankle_p] " + formatVector(
                    dotProd(q_decom(rightAnkleQ_p, [0, -0.707, -0.707]), [-1, 1, 1])) + "\n")
                g.write(
                    "tp @s[tag=r_toe_p] " + formatVector(dotProd(q_decom(rightToeQ_p, [0, 0, -1]), [-1, 1, 1])) + "\n")

                g.write(
                    "tp @s[tag=l_leg_p] " + formatVector(dotProd(q_decom(leftLegQ_p, [0, -1, 0]), [-1, 1, 1])) + "\n")
                g.write(
                    "tp @s[tag=l_knee_p] " + formatVector(dotProd(q_decom(leftKneeQ_p, [0, -1, 0]), [-1, 1, 1])) + "\n")
                g.write("tp @s[tag=l_ankle_p] " + formatVector(
                    dotProd(q_decom(leftAnkleQ_p, [0, -0.707, -0.707]), [-1, 1, 1])) + "\n")
                g.write(
                    "tp @s[tag=l_toe_p] " + formatVector(dotProd(q_decom(leftToeQ_p, [0, 0, -1]), [-1, 1, 1])) + "\n")

                # stop the model from teleporting if the animation makes it teleport for some reason
                if (currentTick != 0 and abs(vector_modulus(
                        [lastPos[0] - position_i[0], lastPos[1] - position_i[1], lastPos[2] - position_i[2]])) > 1):
                    for i in range(3):
                        positionShift[i] += lastPos[i] - position_i[i]
                lastPos = position_i.copy()
                position_final = [0, 0, 0]
                for i in range(3):
                    position_final[i] = position_i[i] + positionShift[i]

                g.write("tp @s[tag=parent] " + formatVector(dotProd(position_final, [-2, -2, 2])) + "\n")

                # rotation to help the legs
                rotationHelp = qToE(groove_i, 'yxz')
                groovePosition = q_decom(groove_i, [0, 1, 0])

                for i in range(3):
                    groovePosition[i] += groovePos[i]
                g.write("tp @s[tag=groove] " + formatVector(dotProd(groovePosition, [2, 2, 2])) + "\n")
                g.write("tp @s[tag=grooveRot] ~ ~ ~ ~" + str(rotationHelp[1]) + " 0\n")
                g.write("tp @s[tag=upperbody_p] " + formatVector(
                    dotProd(q_decom(upperBody_p, [0, 1, 0]), [-1, 1, 1])) + "\n")
                g.write("tp @s[tag=upperbody2_p] " + formatVector(
                    dotProd(q_decom(upperBody2_p, [0, 1, 0]), [-1, 1, 1])) + "\n")

                g.write("tp @s[tag=neck_p] " + formatVector(dotProd(q_decom(neck_p, [0, 1, 0]), [-1, 1, 1])) + "\n")
                g.write("tp @s[tag=head_p] " + formatVector(dotProd(q_decom(head_p, [0, 1, 0]), [-1, 1, 1])) + "\n")
                if currentTick == mcFrameCount - 3:
                    g.write("scoreboard players set @e[type=armor_stand,tag=thisone] tick 0\n")

                g.close()

        # print("new")
        # print(translation[name])
        # print(framecount)`

        bonepos[0] = float(str(struct.unpack('f', f.read(4))[0]).replace(',', ''))
        bonepos[1] = float(str(struct.unpack('f', f.read(4))[0]).replace(',', ''))
        bonepos[2] = float(str(struct.unpack('f', f.read(4))[0]).replace(',', ''))
        # print(bonepos)

        bonerot[0] = float(str(struct.unpack('f', f.read(4))[0]).replace(',', ''))
        bonerot[1] = float(str(struct.unpack('f', f.read(4))[0]).replace(',', ''))
        bonerot[2] = float(str(struct.unpack('f', f.read(4))[0]).replace(',', ''))
        bonerot[3] = float(str(struct.unpack('f', f.read(4))[0]).replace(',', ''))
        # print(bonerot)
        try:
            if translation[name] == "shoulder L":
                current.leftShoulderQ = bonerot.copy()
            elif translation[name] == "arm L":
                current.leftArmQ = bonerot.copy()
            elif translation[name] == "elbow L":
                current.leftElbowQ = bonerot.copy()
            elif translation[name] == "wrist L":
                current.leftWristQ = bonerot.copy()
            elif translation[name] == "shoulder R":
                current.rightShoulderQ = bonerot.copy()
            elif translation[name] == "arm R":
                current.rightArmQ = bonerot.copy()
            elif translation[name] == "elbow R":
                current.rightElbowQ = bonerot.copy()
            elif translation[name] == "wrist R":
                current.rightWristQ = bonerot.copy()
            elif translation[name] == "leg R":
                current.rightLegQ = bonerot.copy()
            elif translation[name] == "knee R":
                current.rightKneeQ = bonerot.copy()
            elif translation[name] == "ankle R":
                current.rightAnkleQ = bonerot.copy()
            elif translation[name] == "toe R":
                current.rightToeQ = bonerot.copy()
            elif translation[name] == "leg L":
                current.leftLegQ = bonerot.copy()
            elif translation[name] == "knee L":
                current.leftKneeQ = bonerot.copy()
            elif translation[name] == "ankle L":
                current.leftAnkleQ = bonerot.copy()
            elif translation[name] == "toe L":
                current.leftToeQ = bonerot.copy()
            elif translation[name] == "upper body":
                current.upperBodyQ = bonerot.copy()
            elif translation[name] == "upper body2":
                current.upperBody2Q = bonerot.copy()
            elif translation[name] == "center":
                current.pos = bonepos.copy()
            elif translation[name] == "groove":
                current.grooveQ = bonerot.copy()
                current.groovePos = bonepos.copy()
            elif translation[name] == "waist":
                current.waistQ = bonerot.copy()
            elif translation[name] == "neck":
                current.neckQ = bonerot.copy()
            elif translation[name] == "head":
                current.headQ = bonerot.copy()
        except:
            print("unknown: " + str(name))

        (binascii.b2a_hex(f.read(64)))


    def binaryTreeConstructor(binaryTreeChildren, binaryLength, binaryDepth):
        treeLength = len(binaryTreeChildren)
        remainderExists = False
        remainder = ""
        if treeLength % 2 == 1:
            remainder = binaryTreeChildren.pop()
            remainderExists = True
            treeLength -= 1
        binaryTreeChildrenNext = []
        for i in range(treeLength >> 1):
            fileNumber = bin(i)[2:].zfill(binaryLength)
            binaryTreeChildrenNext.append(fileNumber)
            completeName = os.path.join(output_directory, str(fileNumber) + ".mcfunction")
            open(completeName, 'w').close()
            g = open(completeName, "a")
            half = i << 1

            start = i * ((2 << binaryDepth) >> 1)
            middle = start + ((1 << binaryDepth) >> 1) - 1
            end = (i + 1) * ((2 << binaryDepth) >> 1) - 1
            if start == middle:
                g.write("execute if score @s tick matches " + str(
                    start) + " run function mmd:" + lastFolderName + "/" + str(binaryTreeChildren[half]) + "\n")
            else:
                g.write("execute if score @s tick matches " + str(start) + ".." + str(
                    middle) + " run function mmd:" + lastFolderName + "/" + str(binaryTreeChildren[half]) + "\n")

            if middle + 1 == end:
                g.write("execute if score @s tick matches " + str(
                    middle + 1) + " run function mmd:" + lastFolderName + "/" + str(
                    binaryTreeChildren[half + 1]) + "\n")
            else:
                g.write("execute if score @s tick matches " + str(middle + 1) + ".." + str(
                    end) + " run function mmd:" + lastFolderName + "/" + str(binaryTreeChildren[half + 1]) + "\n")

            g.close()
        if remainderExists:
            binaryTreeChildrenNext.append(remainder)

        return [binaryTreeChildrenNext, binaryLength - 1, binaryDepth + 1]


    binaryDepth = 1
    while binaryLength > 0:
        binaryTreeChildren, binaryLength, binaryDepth = binaryTreeConstructor(binaryTreeChildren, binaryLength,
                                                                              binaryDepth)

    f.close()
g.close()
