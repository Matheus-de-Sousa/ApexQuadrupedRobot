class Leg:
    FemurLength = 0
    TibiaLength = 0
    SholderOffset = 0

    def __init__(self, femurlength, tibialength, shoulderoffset):
        self.FemurLength = femurlength
        self.TibiaLength = tibialength
        self.SholderOffset = shoulderoffset
    
    def setJointAngle(self, foot, leg, shoulder):
        #Acionamento do servo