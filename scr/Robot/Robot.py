from Leg import Leg
class Apex:
    FrontLeftLeg = Leg()
    FrontRightLeg = Leg()
    BackRightLeg = Leg()
    BackLeftLeg = Leg()

    def __init__(self):
        pass

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