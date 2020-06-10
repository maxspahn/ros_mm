import numpy as np
import diffDrive_MPC_py

def solve_MPC_problem(curState, curU, goal):
    """Solves the MPC problem for the current state

    :curState: actual robot state
    :returns: x_exp expected next state
              u_opt optimal control input for next timestep

    """
    time_horizon = 15
    start = curState;
    start_u = curU
    dt = 0.1
    xinit = np.concatenate((start, start_u))
    x0 = np.tile(xinit, time_horizon)
    setup = np.array([dt, 0.1, 0.5])
    goal = goal
    obstacle = np.array([30, 30, 30, 0.1])
    singleParam = np.concatenate((setup, goal, obstacle))
    params = np.tile(singleParam, time_horizon)
    PARAMS = {}
    PARAMS['xinit'] = xinit
    PARAMS['x0'] = x0
    PARAMS['all_parameters'] = params


    a = diffDrive_MPC_py.diffDrive_MPC_solve(PARAMS)
    u = a[0]['x02']
    x_exp = a[0]['x02'][0:3]
    u_opt = a[0]['x02'][3:5]
    return [x_exp, u_opt]


if __name__ == "__main__":


    curState = np.array([0, 0, 0])
    curCommanded = np.array([0, 0])
    goal = np.array([2, 2, 0])
    for i in range(500):
        [x_exp, u_opt] = solve_MPC_problem(curState, curCommanded, goal)
        curCommanded = u_opt
        curState = x_exp

        print(curCommanded)
        print(curState)


