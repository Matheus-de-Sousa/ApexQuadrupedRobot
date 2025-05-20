from adafruit_servokit import ServoKit
class Leg:
    Joints = ServoKit(channels=16)
    FemurLength = 0
    TibiaLength = 0
    SholderOffset = 0

    # Canais dos sevos das juntas no driver
    footJointChannel = 0
    legJointChannel = 0
    shoulderJointChannel = 0

    def __init__(self, femurlength, tibialength, shoulderoffset, JointsChannels):
        self.FemurLength = femurlength
        self.TibiaLength = tibialength
        self.SholderOffset = shoulderoffset

        self.footJointChannel = JointsChannels[0]
        self.legJointChannel = JointsChannels[1]
        self.shoulderJointChannel = JointsChannels[2]
    
    def setJointAngle(self, foot, leg, shoulder):
        self.Joints.servo[self.footJointChannel].angle = foot
        self.Joints.servo[self.legJointChannel].angle = leg
        self.Joints.servo[self.shoulderJointChannel].angle = shoulder