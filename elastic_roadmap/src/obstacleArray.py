import numpy as np

class ObstacleArray(object):

    """Docstring for ObstacleArray. """

    def __init__(self, nbObstacles):
        self.nbObstacles = nbObstacles
        self.obstacles = np.tile(np.ones(4) * -100, (nbObstacles, 1))

    def setObstacle(self, index, description):
        self.obstacles[index] = description

    def getObstacle(self, index):
        return self.obstacles[index]

    def asVector(self):
        return self.obstacles.flatten()


def genDefaultObstacles():
    myObs = ObstacleArray(5)
    myObs.setObstacle(0, np.array([5, 5, 0, 1]))
    myObs.setObstacle(1, np.array([1, 4, 0, 1]))
    myObs.setObstacle(2, np.array([-5, 1, 0, 1]))
    myObs.setObstacle(3, np.array([2.5, -4, 0, 2]))
    myObs.setObstacle(4, np.array([0, 2, 2, 1]))
    return myObs

def genSimpleObstacles():
    myObs = ObstacleArray(5)
    myObs.setObstacle(0, np.array([2, 2.3, 1, 0.5]))
    return myObs

def genEmptyObstacles():
    myObs = ObstacleArray(5)
    return myObs

if __name__ == "__main__":
    myObs = ObstacleArray(5)
    myObs.setObstacle(12, np.array([1, 1, 0, 0.5]))
    print(myObs.getObstacle(14))
    print(myObs.getObstacle(12))



