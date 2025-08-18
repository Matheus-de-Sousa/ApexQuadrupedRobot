#!/usr/bin/env python3

from Leg import Leg
class Apex:

    def __init__(self):
        self.FrontLeftLeg = Leg(104,131,36, JointsChannels=[0,1,2], offsets=[0,0,0])
        self.BackLeftLeg = Leg(104,131,56, JointsChannels=[3,4,5], offsets=[0,0,0])
        self.FrontRightLeg = Leg(104,131,56, JointsChannels=[6,7,8], offsets=[0,0,0])
        self.BackRightLeg = Leg(104,131,56, JointsChannels=[9,10,11], offsets=[0,0,0])

    def setFrontLeftLeg(self, foot, leg, shoulder):
        LegJointAngle = 180-(90-leg)
        ShoulderJointAngle = 180-shoulder
        self.FrontLeftLeg.setJointAngles(foot,LegJointAngle,ShoulderJointAngle)
    
    def setBackLeftLeg(self, foot, leg, shoulder):
        LegJointAngle = 180-(90-leg)
        ShoulderJointAngle = 180-shoulder
        self.BackLeftLeg.setJointAngles(foot,LegJointAngle,ShoulderJointAngle)

    def setFrontRightLeg(self, foot, leg, shoulder):
        LegJointAngle = 180-(leg+90)
        FootJointAngle = 180-foot
        self.FrontRightLeg.setJointAngles(FootJointAngle,LegJointAngle,shoulder)
    
    
    def setBackRightLeg(self, foot, leg, shoulder):
        LegJointAngle = 180-(leg+90)
        FootJointAngle = 180-foot
        self.BackRightLeg.setJointAngles(FootJointAngle,LegJointAngle,shoulder)