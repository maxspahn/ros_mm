from elastic_map import DynamicGraph
from MPCConnectorMM import MPCConnectorMM
from mapLoader import DynamicGraphLoader

import rospy
import time
import numpy as np
from std_msgs.msg import Float64MultiArray

class ElasticPlanner(object):

    """Docstring for ElasticPlanner. """

    def __init__(self, dynamicMap):
        self._dynamicMap = dynamicMap
        rospy.init_node("ElasticPlanner")
        self._rate = rospy.Rate(10)
        self._pub = rospy.Publisher("/mmrobot/mpc_goal", Float64MultiArray, queue_size=10)
        self._sub = rospy.Subscriber("/mmrobot/curState", Float64MultiArray, self.curState_cb)


    def curState_cb(self, curState):
        self._curState = np.array(curState.data)

    def plan(self, config_a, config_b):
        node_a = self._dynamicMap.addNode(config_a)
        self._dynamicMap.findAndSetConnections(node_a, maxConnections=1)
        node_b = self._dynamicMap.addNode(config_b)
        self._dynamicMap.findAndSetConnections(node_b, maxConnections=1)
        [length, path] = self._dynamicMap.findPath(node_a, node_b)
        return path

    def publishPlan(self, path):
        f = Float64MultiArray()
        for state in path:
            f.data = self._dynamicMap.getConfig(state)
            for i in range(10):
                self._pub.publish(f)
                print("Intermediat Goal published")
                print(f)
                time.sleep(0.5)
            time.sleep(1)

if __name__ == "__main__":
    # Load data
    dgl = DynamicGraphLoader('../savedMaps/testGraph')
    dg = dgl.loadDynamicGraph()
    myPlanner = ElasticPlanner(dg)
    try:
        time.sleep(1)
        print("Starting Planning")
        start_config =  myPlanner._curState
        goal_config = np.array([7.3, 23, 2.4, 1.5, 0.5, 0.6, -1.9, -1.0, 2.28, 2.2])
        print(start_config)
        path = myPlanner.plan(start_config, goal_config)
        myPlanner.publishPlan(path)
    except rospy.ROSInterruptException:
        pass
