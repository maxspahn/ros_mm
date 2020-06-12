import numpy as np

class EuclideanConnector(object):

    def __init__(self, name):
        self.name = name

    def dist(self, config1, config2):
        return np.linalg.norm(config2 - config1)
