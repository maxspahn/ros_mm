import numpy as np

class RobotModel(object):

    """Docstring for ObstacleArray. """

    def __init__(self, name):
        self.name = name
        if name == "mm":
            self.createMM()

    def createMM(self):
        self.jointLimitsUp = np.array([15, 15, 3.14, 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525,
2.8973])
        self.jointLimitsLow = np.array([-15, -15, -3.14, -2.8973, -1.7628, -2.8973, -3.0718, -2.8973,
-0.0175, -2.8973])
        self.jointVelLimits = np.array([2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100,
2.6100])
        self.jointTorqueLimits = np.array([87, 87, 87, 87, 12, 12, 12])
        self.jointAccLimits = np.array([15, 7.5, 10, 12.5, 15, 20, 20, 20])
        self.jointMeans = 0.5 * (self.jointLimitsUp + self.jointLimitsLow)
        self.jointStds = 0.25 * (self.jointLimitsUp - self.jointLimitsLow)

    def getMeans(self):
        return self.jointMeans

    def getSigmas(self):
        return self.jointStds

    def isValidConfig(self, config):
        lowCheck = config > self.jointLimitsLow
        upCheck = config < self.jointLimitsUp
        return (upCheck.all() and lowCheck.all())

if __name__ == "__main__":
    myRobotModel = RobotModel("mm")
    print(myRobotModel.getMeans())



