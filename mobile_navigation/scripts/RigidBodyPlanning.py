#!/usr/bin/env python

try:
    from ompl import base as ob
    from ompl import geometric as og
except ImportError:
    # if the ompl module is not in the PYTHONPATH assume it is installed in a
    # subdirectory of the parent directory called "py-bindings."
    from os.path import abspath, dirname, join
    import sys
    sys.path.insert(0, join(dirname(dirname(abspath(__file__))), 'py-bindings'))
    from ompl import base as ob
    from ompl import geometric as og

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def isStateValid(state):
    # Some arbitrary condition on the state (note that thanks to
    # dynamic type checking we can just call getX() and do not need
    # to convert state to an SE2State.)
    return state.getX() < .6

def planWithSimpleSetup():
    # create an SE2 state space
    space = ob.SE2StateSpace()

    # set lower and upper bounds
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-1)
    bounds.setHigh(1)
    space.setBounds(bounds)

    # create a simple setup object
    ss = og.SimpleSetup(space)
    ss.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))

    start = ob.State(space)
    # we can pick a random start state...
    start.random()
    # ... or set specific values
    start().setX(.5)

    goal = ob.State(space)
    # we can pick a random goal state...
    goal.random()
    # ... or set specific values
    goal().setX(-.5)

    ss.setStartAndGoalStates(start, goal)

    # this will automatically choose a default planner with
    # default parameters
    solved = ss.solve(1.0)

    if solved:
        # try to shorten the path
        ss.simplifySolution()
        # print the simplified path
        print(ss.getSolutionPath())


def planTheHardWay():
    # create an SE2 state space
    space = ob.SE2StateSpace()
    # set lower and upper bounds
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(-1)
    bounds.setHigh(1)
    space.setBounds(bounds)
    # construct an instance of space information from this state space
    si = ob.SpaceInformation(space)
    # set state validity checking for this space
    si.setStateValidityChecker(ob.StateValidityCheckerFn(isStateValid))
    # create a random start state
    start = ob.State(space)
    start.random()
    # create a random goal state
    goal = ob.State(space)
    goal.random()
    # create a problem instance
    pdef = ob.ProblemDefinition(si)
    # set the start and goal states
    pdef.setStartAndGoalStates(start, goal)
    # create a planner for the defined space
    planner = og.RRTConnect(si)
    # set the problem we are trying to solve for the planner
    planner.setProblemDefinition(pdef)
    # perform setup steps for the planner
    planner.setup()
    # print the settings for this space
    print(si.settings())
    # print the problem settings
    print(pdef)
    # attempt to solve the problem within one second of planning time
    solved = planner.solve(1.0)

    if solved:
        # get the goal representation from the problem definition (not the same as the goal state)
        # and inquire about the found path
        path = pdef.getSolutionPath()
        return path
        print("Found solution:\n%s" % path)
    else:
        print("No solution found")

def movebase_client(x, w):

    rospy.init_node('movebase_client_py')
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    print("Waiting for server to become available")
    client.wait_for_server()

    goal = MoveBaseGoal()
    print(goal)
    goal.target_pose.header.frame_id = "map"
    for i in range(3):
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x[i]
        goal.target_pose.pose.position.y = y[i]
        goal.target_pose.pose.orientation.w = 1

        print(goal)
        client.send_goal(goal)
        print("the goal was sent to the server")
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            print(client.get_result())



if __name__ == "__main__":
    #planWithSimpleSetup()
    print("")
    p = planTheHardWay()
    p.interpolate(10)
    pM = p.printAsMatrix()
    print(pM)
    p1 = p.getState(0)
    print(p1.printState())
    #print(x)
    #x = [0.5, 3.0, 3.5]
    #y = [3.0, 3.0, 1.0]
'''
    try:
        result = movebase_client(x, y)
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
'''
"""
    planWithSimpleSetup()
    print("")
    planTheHardWay()
"""
