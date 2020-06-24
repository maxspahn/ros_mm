import numpy as np
from params import ParametersElasticMap

class EuclideanConnector(object):

    def __init__(self, name, params=ParametersElasticMap()):
        self.name = name
        self.maxSpeed = params._maxSpeed

    def dist(self, config1, config2):
        return (np.linalg.norm(config2 - config1)/self.maxSpeed)
