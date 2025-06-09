from Leg import Leg
class Apex:

    def __init__(self):
        self.FrontLeftLeg = Leg()
        self.FrontRightLeg = Leg()
        self.BackRightLeg = Leg()
        self.BackLeftLeg = Leg()

    def setFrontLeftLeg(self, foot, leg, shoulder):
        LegJointAngle = 180-(90-leg)
        ShouldeJointAngle = 180-shoulder
        self.FrontLeftLeg.setJointAngles(foot,LegJointAngle,ShouldeJointAngle)
    
    def setFrontRightLeg(self, foot, leg, shoulder):
        LegJointAngle = 180-(leg+90)
        FootJointAngle = 180-foot
        self.FrontLeftLeg.setJointAngles(FootJointAngle,LegJointAngle,shoulder)
    
    def setBackLeftLeg(self, foot, leg, shoulder):
        LegJointAngle = 180-(90-leg)
        ShouldeJointAngle = 180-shoulder
        self.FrontLeftLeg.setJointAngles(foot,LegJointAngle,ShouldeJointAngle)
    
    def setBackRightLeg(self, foot, leg, shoulder):
        LegJointAngle = 180-(leg+90)
        FootJointAngle = 180-foot
        self.FrontLeftLeg.setJointAngles(FootJointAngle,LegJointAngle,shoulder)