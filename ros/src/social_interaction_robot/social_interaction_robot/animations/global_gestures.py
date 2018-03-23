# Choregraphe simplified export in Python.
from naoqi import ALProxy

class GlobalGestures():
    
    def __init__(self):
        self.initializeGlobalGestures()        
        
    def getMotion(self, motion):
        if motion == "InitPostureArms": 
            names = self.namesInitPostureArms 
            keys = self.keysInitPostureArms 
            times = self.timesInitPostureArms
        elif motion == "InitPosture": 
            names = self.namesInitPosture 
            keys = self.keysInitPosture 
            times = self.timesInitPosture
        return [names, keys, times]
    
    def initializeGlobalGestures(self):
        self.initializeInitPostureArms()
        self.initializeInitPosture()
        
    def initializeInitPostureArms(self):
        self.namesInitPostureArms = list()
        self.timesInitPostureArms = list()
        self.keysInitPostureArms = list()
        
        self.namesInitPostureArms.append("LElbowRoll")
        self.timesInitPostureArms.append([1.56])
        self.keysInitPostureArms.append([-0.52002])
        
        self.namesInitPostureArms.append("LElbowYaw")
        self.timesInitPostureArms.append([1.56])
        self.keysInitPostureArms.append([-1.22718])
        
        self.namesInitPostureArms.append("LHand")
        self.timesInitPostureArms.append([1.56])
        self.keysInitPostureArms.append([0.584359])
        
        self.namesInitPostureArms.append("LShoulderPitch")
        self.timesInitPostureArms.append([1.56])
        self.keysInitPostureArms.append([1.55852])
        
        self.namesInitPostureArms.append("LShoulderRoll")
        self.timesInitPostureArms.append([1.56])
        self.keysInitPostureArms.append([0.145728])
        
        self.namesInitPostureArms.append("LWristYaw")
        self.timesInitPostureArms.append([1.56])
        self.keysInitPostureArms.append([0.0352399])
        
        self.namesInitPostureArms.append("RElbowRoll")
        self.timesInitPostureArms.append([1.56])
        self.keysInitPostureArms.append([0.529223])
        
        self.namesInitPostureArms.append("RElbowYaw")
        self.timesInitPostureArms.append([1.56])
        self.keysInitPostureArms.append([1.22412])
        
        self.namesInitPostureArms.append("RHand")
        self.timesInitPostureArms.append([1.56])
        self.keysInitPostureArms.append([0.608084])
        
        self.namesInitPostureArms.append("RShoulderPitch")
        self.timesInitPostureArms.append([1.56])
        self.keysInitPostureArms.append([1.55546])
        
        self.namesInitPostureArms.append("RShoulderRoll")
        self.timesInitPostureArms.append([1.56])
        self.keysInitPostureArms.append([-0.141126])
        
        self.namesInitPostureArms.append("RWristYaw")
        self.timesInitPostureArms.append([1.56])
        self.keysInitPostureArms.append([0.0229681])
    
    def initializeInitPosture(self):
        self.namesInitPosture = list()
        self.timesInitPosture = list()
        self.keysInitPosture = list()
        
        self.namesInitPosture.append("HeadPitch")
        self.timesInitPosture.append([0.96])
        self.keysInitPosture.append([-0.268447])
        
        self.namesInitPosture.append("HeadYaw")
        self.timesInitPosture.append([0.96])
        self.keysInitPosture.append([0.0276117])
        
        self.namesInitPosture.append("LElbowRoll")
        self.timesInitPosture.append([0.96])
        self.keysInitPosture.append([-0.521554])
        
        self.namesInitPosture.append("LElbowYaw")
        self.timesInitPosture.append([0.96])
        self.keysInitPosture.append([-1.23179])
        
        self.namesInitPosture.append("LHand")
        self.timesInitPosture.append([0.96])
        self.keysInitPosture.append([0.586995])
        
        self.namesInitPosture.append("LShoulderPitch")
        self.timesInitPosture.append([0.96])
        self.keysInitPosture.append([1.55392])
        
        self.namesInitPosture.append("LShoulderRoll")
        self.timesInitPosture.append([0.96])
        self.keysInitPosture.append([0.148796])
        
        self.namesInitPosture.append("LWristYaw")
        self.timesInitPosture.append([0.96])
        self.keysInitPosture.append([0.0214341])
        
        self.namesInitPosture.append("RElbowRoll")
        self.timesInitPosture.append([0.96])
        self.keysInitPosture.append([0.529223])
        
        self.namesInitPosture.append("RElbowYaw")
        self.timesInitPosture.append([0.96])
        self.keysInitPosture.append([1.21798])
        
        self.namesInitPosture.append("RHand")
        self.timesInitPosture.append([0.96])
        self.keysInitPosture.append([0.594903])
        
        self.namesInitPosture.append("RShoulderPitch")
        self.timesInitPosture.append([0.96])
        self.keysInitPosture.append([1.56313])
        
        self.namesInitPosture.append("RShoulderRoll")
        self.timesInitPosture.append([0.96])
        self.keysInitPosture.append([-0.147262])
        
        self.namesInitPosture.append("RWristYaw")
        self.timesInitPosture.append([0.96])
        self.keysInitPosture.append([0.053648])




        

