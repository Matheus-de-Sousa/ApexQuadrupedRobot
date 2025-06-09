#!/usr/bin/env python3

import rospy
import math
from apex_controller.inverse_kinematics import InverseKinematics, interpolate, KeyframeKinematics

from adafruit_servokit import ServoKit
RobotJoints = ServoKit(channels=16)

class locomotion_controller(object):

    def __init__(self):

        self.rate = rospy.Rate(200)
        #self.gaits = [[100,750,160], [100,550,160], [-100,550,160], [-100,750,160],[-40,750,160],[40,750,160]]
        self.gaits = [[-30,220,60], [-30,190,60], [30,190,60], [30,220,60],[18,220,60],[6,220,60],[-6,220,60],[-18,220,60]]
        self.lastGaitIndex = 0
        self.lastElapsedTime = 0

        self.forward_factor = 1.1
        self.height_factor = -15
        self.rotation_factor = 0

        self.keyframesFrontLeftLeg = [[-30,210, 60], [-30,200, 70]]
        self.keyframesBackLeftLeg = [[-30,210, 60], [-30,200, 70]]
        self.keyframesFrontRightLeg = [[-30,210, 60], [-30,200, 70]]
        self.keyframesBackRightLeg = [[-30,210, 60], [-30,200, 70]]

        self.stand()

    def LocomotionRun(self, elapsedSequenceTime,sequenceTime):
        pass
    def UpdateMovementSequence(self, elapsedSequenceTime,sequenceTime):
        ratio = float((elapsedSequenceTime-self.lastElapsedTime)/sequenceTime)
        gaitIndex = int(ratio)
        if(self.lastGaitIndex != gaitIndex):
            self.ShiftKeyframe()

        angle = 45.0/180.0*math.pi
        x_rot = math.sin(angle) * self.rotation_factor
        z_rot = math.cos(angle) * self.rotation_factor

        angle = (45+self.gaits[gaitIndex][0])/180.0*math.pi
        x_rotFR = x_rot-math.sin(angle) * self.rotation_factor
        z_rotFR = z_rot-math.cos(angle) * self.rotation_factor

        self.keyframesFrontRightLeg[1] = self.gaits[gaitIndex].copy()
        self.keyframesFrontRightLeg[1][1] += self.height_factor
        self.keyframesFrontRightLeg[1][0] = self.keyframesFrontRightLeg[1][0]*self.forward_factor + x_rotFR
        self.keyframesFrontRightLeg[1][2] += z_rotFR 

        adjusted_index2 = gaitIndex + 2
        if(adjusted_index2 >= len(self.gaits)):
            adjusted_index2 -= len(self.gaits)

        angle = (45+self.gaits[adjusted_index2][0])/180.0*math.pi
        x_rotBR = x_rot-math.sin(angle) * self.rotation_factor
        z_rotBR = z_rot-math.cos(angle) * self.rotation_factor

        self.keyframesBackRightLeg[1] = self.gaits[adjusted_index2].copy()
        self.keyframesBackRightLeg[1][1] += self.height_factor
        self.keyframesBackRightLeg[1][0] = self.keyframesBackRightLeg[1][0]*self.forward_factor + x_rotBR
        self.keyframesBackRightLeg[1][2] += -z_rotBR 

        adjusted_index3 = gaitIndex + 4
        if(adjusted_index3 >= len(self.gaits)):
            adjusted_index3 -= len(self.gaits)

        angle = (45+self.gaits[adjusted_index3][0])/180.0*math.pi
        x_rotFL = x_rot-math.sin(angle) * self.rotation_factor
        z_rotFL = z_rot-math.cos(angle) * self.rotation_factor

        self.keyframesFrontLeftLeg[1] = self.gaits[adjusted_index3].copy()
        self.keyframesFrontLeftLeg[1][1] += self.height_factor
        self.keyframesFrontLeftLeg[1][0] = self.keyframesFrontLeftLeg[1][0]*self.forward_factor - x_rotFL
        self.keyframesFrontLeftLeg[1][2] += -z_rotFL

        
        adjusted_index4 = gaitIndex + 6
        if(adjusted_index4 >= len(self.gaits)):
            adjusted_index4 -= len(self.gaits)
        
        angle = (45+self.gaits[adjusted_index4][0])/180.0*math.pi
        x_rotBL = x_rot-math.sin(angle) * self.rotation_factor
        z_rotBL = z_rot-math.cos(angle) * self.rotation_factor

        self.keyframesBackLeftLeg[1] = self.gaits[adjusted_index4].copy()
        self.keyframesBackLeftLeg[1][1] += self.height_factor
        self.keyframesBackLeftLeg[1][0] = self.keyframesBackLeftLeg[1][0]*self.forward_factor - x_rotBL
        self.keyframesBackLeftLeg[1][0] += z_rotBL
        '''self.keyframesFrontRightLeg[1] = self.gaits[gaitIndex]
        self.keyframesBackLeftLeg[1] = self.gaits[gaitIndex]

        adjusted_index = gaitIndex + int(len(self.gaits)/2)
        if(adjusted_index >= len(self.gaits)):
            adjusted_index -= len(self.gaits)
        self.keyframesFrontLeftLeg[1] = self.gaits[adjusted_index]
        self.keyframesBackRightLeg[1] = self.gaits[adjusted_index]'''
        
        if(adjusted_index4 == 1 or adjusted_index3 == 1 or adjusted_index4 == 2 or adjusted_index3 == 2):
            self.keyframesBackLeftLeg[1][2] = 60+20
            self.keyframesFrontLeftLeg[1][2] = 60+20
            self.keyframesBackRightLeg[1][2] = 60-20
            self.keyframesFrontRightLeg[1][2] = 60-20
        else:
            self.keyframesBackLeftLeg[1][2] = 60+20
            self.keyframesFrontLeftLeg[1][2] = 60+20
            self.keyframesBackRightLeg[1][2] = 60-20
            self.keyframesFrontRightLeg[1][2] = 60-20
        ratio = ratio - gaitIndex
        self.UpdateLegsPosition(ratio)

        if(gaitIndex >= len(self.gaits) - 1):
            self.lastElapsedTime = elapsedSequenceTime
        self.lastGaitIndex = gaitIndex

        self.rate.sleep()
    
    def UpdateLegsPosition(self, ratio):
        key1 = self.keyframesFrontLeftLeg[0]
        key2 = self.keyframesFrontLeftLeg[1]

        foot, leg, shoulder = KeyframeKinematics(key1, key2, ratio)
        self.moveFrontLeftLeg(foot, leg, shoulder)

        key1 = self.keyframesBackLeftLeg[0]
        key2 = self.keyframesBackLeftLeg[1]

        foot, leg, shoulder = KeyframeKinematics(key1, key2, ratio)
        self.moveBackLeftLeg(foot, leg, shoulder)

        key1 = self.keyframesFrontRightLeg[0]
        key2 = self.keyframesFrontRightLeg[1]

        foot, leg, shoulder = KeyframeKinematics(key1, key2, ratio)
        self.moveFrontRightLeg(foot, leg, shoulder)

        key1 = self.keyframesBackRightLeg[0]
        key2 = self.keyframesBackRightLeg[1]

        foot, leg, shoulder = KeyframeKinematics(key1, key2, ratio)
        self.moveBackRightLeg(foot, leg, shoulder)
    
    def ShiftKeyframe(self):
        self.keyframesFrontLeftLeg[0] = self.keyframesFrontLeftLeg[1]
        self.keyframesFrontRightLeg[0] = self.keyframesFrontRightLeg[1]
        self.keyframesBackLeftLeg[0] = self.keyframesBackLeftLeg[1]
        self.keyframesBackRightLeg[0] = self.keyframesBackRightLeg[1]
    
    def stand(self):
        self.moveFrontLeftLeg(90, 0, 90)
        self.moveFrontRightLeg(90, 0, 90)
        self.moveBackLeftLeg(90, 0, 90)
        self.moveBackRightLeg(90, 0, 90)

        self.rate.sleep()

    def land(self):
        self.rate.sleep()

    def up(self, angleStep):
        self.rate.sleep()
    
    def moveFrontLeftLeg(self, foot, leg, shoulder):
        #print([foot,180-(90-leg), 180-shoulder])
        RobotJoints.servo[0].angle = foot
        RobotJoints.servo[4].angle = 180-(90-leg)
        RobotJoints.servo[5].angle = 180-shoulder
    
    def moveBackLeftLeg(self, foot, leg, shoulder):
        #print([foot,180-(90-leg), 180-shoulder])
        RobotJoints.servo[15].angle = foot
        RobotJoints.servo[11].angle = 180-(90-leg)
        RobotJoints.servo[10].angle = 180-shoulder

    def moveFrontRightLeg(self, foot, leg, shoulder):
        #print([180-foot,180-(90+leg), shoulder])
        RobotJoints.servo[3].angle = 180-foot
        RobotJoints.servo[1].angle = 180-(90+leg)
        RobotJoints.servo[2].angle = shoulder

    def moveBackRightLeg(self, foot, leg, shoulder):
        #print([180-foot,180-(90+leg), shoulder])
        RobotJoints.servo[12].angle = 180-foot
        RobotJoints.servo[14].angle = 180-(90+leg)
        RobotJoints.servo[13].angle = shoulder
