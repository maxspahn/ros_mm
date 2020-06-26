from elastic_map import DynamicGraph
from MPCConnectorMM import MPCConnectorMM
from mapLoader import DynamicGraphLoader

import rospy
import actionlib
import time
import numpy as np
from std_msgs.msg import Float64MultiArray, Float64

import mobile_mpc.msg


class ElasticPlanner(object):

    """Docstring for ElasticPlanner. """

    def __init__(self, dynamicMap):
        self._dynamicMap = dynamicMap
        rospy.init_node("ElasticPlanner")
        self._rate = rospy.Rate(10)
        self._sub = rospy.Subscriber(
            "/mmrobot/curState", Float64MultiArray, self.curState_cb
        )
        self._client = actionlib.SimpleActionClient("mpc", mobile_mpc.msg.mpcAction)
        rospy.loginfo("Waiting for mpc server to come up")
        self._client.wait_for_server()
        self._curState = np.array([4, -8, -1.4, 0.5, 0.5, 0.6, -1.2, -1.0, 2.28, 2.2])
        self._curState = np.array([-1, 6, 2.4, 1.5, 0.5, 0.6, -1.9, -1.0, 2.28, 2.2])
        self._curState = np.array([0, 0, 0, 1.5, 0.5, 0.6, -1.9, -1.0, 2.28, 2.2])

    def curState_cb(self, curState):
        self._curState = np.array(curState.data)

    def plan(self, config_a, config_b):
        node_a = self._dynamicMap.addNode(config_a)
        self._dynamicMap.findAndSetConnections(node_a, maxConnections=1, maxDistBase=5)
        node_b = self._dynamicMap.addNode(config_b)
        self._dynamicMap.findAndSetConnections(node_b, maxConnections=1, maxDistBase=5)
        [length, path] = self._dynamicMap.findPath(node_a, node_b)
        return path

    def publishPlan(self, path):
        state = Float64MultiArray()
        eW = Float64MultiArray()
        eW.data = np.array([1.0, 0.0, 0.5])
        mE = Float64()
        mE.data = 1.5
        mW = Float64MultiArray()
        mW.data = np.array([1.0, 100.0, 0.0, 100000000.0, 0.2, 10.0])
        for i in range(len(path)):
            state.data = self._dynamicMap.getConfig(path[i])
            print("next state : ", state.data)
            if i == (len(path) - 1):
                mE.data = 0.5
                eW.data = np.array([1.0, 1.0, 1.5])
                mW.data = np.array([10.0, 100.0, 10.0, 100000000.0, 1.0, 10.0])
            goal = mobile_mpc.msg.mpcGoal(
                goal=state, errorWeights=eW, maxError=mE, mpcWeights=mW
            )
            self._client.send_goal(goal)
            self._client.wait_for_result()
            print("Reached state : ", self._client.get_result().finalState.data)

    def saveRoadMap(self, fileName):
        self._dynamicMap.saveGraph(fileName)



if __name__ == "__main__":
    # Load data
    fileNameGraph = "../savedMaps/completeGraph"
    dgl = DynamicGraphLoader(fileNameGraph)
    dg = dgl.loadDynamicGraph()
    myPlanner = ElasticPlanner(dg)
    dg.plot(short=False)
    try:
        print("Starting Planning")
        start_config = myPlanner._curState
        print("Start State : ", myPlanner._curState)
        goal_config = np.array([-1, 6, 2.4, 1.5, 0.5, 0.6, -1.9, -1.0, 2.28, 2.2])
        goal_config = np.array([-6, 3, -1.4, 0.5, 0.5, 0.6, -1.2, -1.0, 2.28, 2.2])
        print(start_config)
        path = myPlanner.plan(start_config, goal_config)
        print(path)
        dg.plot(short=False)
        myPlanner.publishPlan(path)
        myPlanner.saveRoadMap("../savedMaps/afterRunning")
    except rospy.ROSInterruptException:
        pass
