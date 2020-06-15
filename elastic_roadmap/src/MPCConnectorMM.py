import numpy as np
import time

from mobile_mpc import solve_MPC_mm
import obstacleArray

class MPCConnectorMM(object):

    def __init__(self, name, time_horizon, maxDist = 20, obstacles=obstacleArray.genDefaultObstacles()):
        self.name = name
        self.time_horizon = time_horizon
        self.dt = 0.1
        self.maxDist = maxDist
        self.obstacles = obstacles
        self.problemSetup()

    def dist(self, config1, config2):
        self.goal = config2
        self.curState = config1
        return self.planConnection()

    def problemSetup(self):
        wheel_radius = 0.08
        wheel_distance = 0.544
        self.curU = np.zeros(9)
        self.setup = np.array([self.dt, wheel_radius, wheel_distance])
        self.PARAMS = {}

    def singleMPCStep(self):
        [x_exp, u_opt] = self.solve()
        self.curU = u_opt
        self.curState = x_exp

    def computeError(self):
        return np.linalg.norm(
            self.curState[0:2] - self.goal[0:2]
        ) + 0.5 * np.linalg.norm(self.curState[3:10] - self.goal[3:10])

    def planConnection(self):
        timeOfTravel = 0
        error = 100
        while True:
            if timeOfTravel >= self.maxDist:
                timeOfTravel = -1
                break
            timeOfTravel += self.dt
            oldError = error
            error = self.computeError()
            if error < 0.4:
                break
            self.singleMPCStep()
        return timeOfTravel

    def solve(self):
        """Solves the MPC problem for the current state

        :curState: actual robot state
        :returns: x_exp expected next state
                  u_opt optimal control input for next timestep

        """
        xinit = np.concatenate((self.curState, self.curU))
        x0 = np.tile(xinit, self.time_horizon)
        singleParam = np.concatenate((self.setup, self.goal, self.obstacles.asVector()))
        params = np.tile(singleParam, self.time_horizon)
        self.PARAMS["xinit"] = xinit
        self.PARAMS["x0"] = x0
        self.PARAMS["all_parameters"] = params
        solution = solve_MPC_mm.solve_MPC_mm(self.PARAMS)
        x02 = solution[0]["x02"]
        x_exp = x02[0:10]
        u_opt = x02[10:19]
        return [x_exp, u_opt]


if __name__ == "__main__":
    config1 = np.zeros(10)
    config1[6] = -1
    config2 = np.zeros(10)
    config2[6] = -1
    config2[0] = 0
    con= MPCConnectorMM("mobileManipulatorMPC", 15)
    dist = con.dist(config1, config2)
    print(dist)
