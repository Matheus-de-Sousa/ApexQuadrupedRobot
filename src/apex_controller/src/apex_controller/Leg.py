#!/usr/bin/env python3

from adafruit_servokit import ServoKit
class Leg:
    
    def __init__(self, femurlength, tibialength, shoulderoffset, JointsChannels):
        self.Joints = ServoKit(channels=16)
        
        self.FemurLength = femurlength
        self.TibiaLength = tibialength
        self.SholderOffset = shoulderoffset

        # Canais dos sevos das juntas no driver
        self.footJointChannel = JointsChannels[0]
        self.legJointChannel = JointsChannels[1]
        self.shoulderJointChannel = JointsChannels[2]
    
    def setJointAngle(self, foot, leg, shoulder):
        self.Joints.servo[self.footJointChannel].angle = foot
        self.Joints.servo[self.legJointChannel].angle = leg
        self.Joints.servo[self.shoulderJointChannel].angle = shoulder