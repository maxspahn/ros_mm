class ParametersElasticMap(object):

    """Docstring for ParametersElasticMap. """

    def __init__(
        self,
        dt=0.2,
        minDistBase=1,
        maxTime=30,
        maxSpeed=0.4,
        timeHorizon=20,
        variancesBase=[2, 2, 0.5],
        varianceFactorJoints=0.5,
        dim=10
    ):
        self._dt = dt
        self._minDistBase = minDistBase
        self._maxTime = maxTime
        self._maxSpeed = maxSpeed
        self._timeHorizon = timeHorizon
        self._variancesBase = variancesBase
        self._varianceFactorJoints = varianceFactorJoints
        self._dim = dim
