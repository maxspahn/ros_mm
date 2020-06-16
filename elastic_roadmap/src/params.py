class ParametersElasticMap(object):

    """Docstring for ParametersElasticMap. """

    def __init__(
        self,
        dt=0.5,
        minDistBase=2,
        maxDist=20,
        maxSpeed=0.4,
        timeHorizon=15,
        variancesBase=[3.5, 3.5, 0.5],
        varianceFactorJoints=0.5,
        dim=10
    ):
        self._dt = dt
        self._minDistBase = minDistBase
        self._maxDist = maxDist
        self._maxSpeed = maxSpeed
        self._timeHorizon = timeHorizon
        self._variancesBase = variancesBase
        self._varianceFactorJoints = varianceFactorJoints
        self._dim = dim
