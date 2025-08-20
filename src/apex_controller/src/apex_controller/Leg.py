#!/usr/bin/env python3

from adafruit_servokit import ServoKit
class Leg:
    
    def __init__(self, femurlength, tibialength, shoulderoffset, JointsChannels, offsets):
        self.Joints = ServoKit(channels=16)
        
        self.FemurLength = femurlength
        self.TibiaLength = tibialength
        self.SholderOffset = shoulderoffset

        # Canais dos sevos das juntas no driver
        self.footJointChannel = JointsChannels[0]
        self.legJointChannel = JointsChannels[1]
        self.shoulderJointChannel = JointsChannels[2]

        self.footJointOffset = offsets[0]
        self.legJointOffset = offsets[1]
        self.shoulderJointOffset = offsets[2]
    
    def setJointsAngles(self, foot, leg, shoulder):
        self.Joints.servo[self.footJointChannel].angle = foot + self.footJointOffset
        self.Joints.servo[self.legJointChannel].angle = leg + self.legJointOffset
        self.Joints.servo[self.shoulderJointChannel].angle = shoulder + self.shoulderJointOffset
