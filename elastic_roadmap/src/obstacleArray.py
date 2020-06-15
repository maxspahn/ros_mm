import numpy as np

class ObstacleArray(object):

    """Docstring for ObstacleArray. """

    def __init__(self, nbObstacles):
        self.nbObstacles = nbObstacles
        self.obstacles = np.tile(np.zeros(4), (nbObstacles, 1))

    def setObstacle(self, index, description):
        self.obstacles[index] = description

    def getObstacle(self, index):
        return self.obstacles[index]

    def asVector(self):
        return self.obstacles.flatten()


def genDefaultObstacles():
    myObs = ObstacleArray(50)
    myObs.setObstacle(0, np.array([5, 5, 2, 1.5]))
    myObs.setObstacle(1, np.array([1, 2, 2, 1.5]))
    myObs.setObstacle(2, np.array([-0.5, 3.5, 2, 1.5]))
    myObs.setObstacle(3, np.array([-2, 5, 2, 1.5]))
    myObs.setObstacle(4, np.array([-3.5, 6.5, 2, 1.5]))
    myObs.setObstacle(5, np.array([-5, 6.5, 2, 1.5]))
    myObs.setObstacle(6, np.array([4, -2, 1, 1.5]))
    myObs.setObstacle(7, np.array([4, -3.5, 1, 1.5]))
    myObs.setObstacle(8, np.array([4, -5, 1, 1.5]))
    myObs.setObstacle(9, np.array([4, -6.5, 1, 1.5]))
    myObs.setObstacle(10, np.array([5.5, -6.5, 1, 1.5]))
    return myObs

def genSimpleObstacles():
    myObs = ObstacleArray(50)
    myObs.setObstacle(0, np.array([2, 2.3, 1, 0.5]))
    return myObs

if __name__ == "__main__":
    myObs = ObstacleArray(50)
    myObs.setObstacle(12, np.array([1, 1, 0, 0.5]))
    print(myObs.getObstacle(14))
    print(myObs.getObstacle(12))



