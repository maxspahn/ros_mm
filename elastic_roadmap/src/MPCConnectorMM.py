import numpy as np
import time

from mobile_mpc import solve_MPC_mm
import obstacleArray
from params import ParametersElasticMap

class MPCConnectorMM(object):

    def __init__(self, name, params=ParametersElasticMap(), obstacles=obstacleArray.genDefaultObstacles()):
        self.name = name
        self.time_horizon = params._timeHorizon
        self.dt = params._dt
        self.maxTime = params._maxTime
        self.maxSpeed = params._maxSpeed
        self.obstacles = obstacles
        self.slack = np.zeros(1)
        self.nbVar = 20
        self.PARAMS = {}
        self.curU = np.zeros(9)
        self.curState = np.zeros(10)
        self.goal = np.zeros(10)
        self.problemSetup()

    def dist(self, config1, config2):
        self.goal = config2
        self.start = config1
        print("Computing distance between")
        print("Start : ", self.start[0] ,self.start[1])
        print("Goal: ", self.goal[0] ,self.goal[1])
        if self.isValidCandidate():
            return self.planConnection()
        else:
            return -1

    def isValidCandidate(self):
        euclDistanceBase = np.linalg.norm(self.goal[0:2] - self.start[0:2])
        if (1.5 * euclDistanceBase/self.maxSpeed) > self.maxTime:
            return False
        return True

    def problemSetup(self):
        wheel_radius = 0.08
        wheel_distance = 0.544
        self.setup = np.array([self.dt, wheel_radius, wheel_distance])
        self.x0 = np.zeros(self.nbVar * self.time_horizon)
        self.xinit = np.zeros(20)
        # q, x, o, slack, u, qdot,
        self.weights = np.array([10.0, 1000.0, 1.0, 100000.0, 1.0, 10.0])

    def singleMPCStep(self):
        [x_exp, u_opt] = self.solve()
        self.curU = u_opt
        self.start= x_exp

    def computeError(self):
        return np.linalg.norm(
            self.start[0:2] - self.goal[0:2]
        ) + 0.5 * np.linalg.norm(self.start[3:10] - self.goal[3:10])

    def planConnection(self):
        timeOfTravel = 0
        error = 100
        while True:
            if timeOfTravel >= self.maxTime:
                timeOfTravel = -1
                break
            timeOfTravel += self.dt
            oldError = error
            error = self.computeError()
            if error < 0.2:
                break
            self.singleMPCStep()
        return timeOfTravel

    def solve(self):
        """Solves the MPC problem for the current state

        :returns: x_exp expected next state
                  u_opt optimal control input for next timestep

        """
        xinit = np.concatenate((self.start, self.slack, self.curU))
        x0 = np.tile(xinit, self.time_horizon)
        singleParam = np.concatenate((self.setup, self.goal, self.weights, self.obstacles.asVector()))
        params = np.tile(singleParam, self.time_horizon)
        self.PARAMS["xinit"] = xinit
        self.PARAMS["x0"] = x0
        self.PARAMS["all_parameters"] = params
        solution = solve_MPC_mm.solve_MPC_mm(self.PARAMS)
        x02 = solution[0]["x02"]
        x_exp = x02[0:10]
        u_opt = x02[11:20]
        return [x_exp, u_opt]


if __name__ == "__main__":
    config1 = np.zeros(10)
    config1[6] = -1
    config2 = np.zeros(10)
    config2[6] = -1
    config2[0] = 0
    con= MPCConnectorMM("mobileManipulatorMPC")
    dist = con.dist(config1, config2)
    print(dist)
