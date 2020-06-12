#!/usr/bin/env python

import os
import sys

sys.path.append(
    os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "../src/"
    )
)

import numpy as np

from EuclideanConnector import EuclideanConnector
from MPCConnectorMM import MPCConnectorMM
from elastic_map import DynamicGraph

def blockPrint():
    sys.stdout = open(os.devnull, 'w')

def enablePrint():
    sys.stdout = sys.__stdout__

if __name__ == "__main__":
    blockPrint()
    eCon = MPCConnectorMM("MPCConnector", 15);
    #eCon = EuclideanConnector("EucCon");
    dg = DynamicGraph(10, eCon, maxDist=50)
    nodeId1 = dg.addNode(np.array([1.2, 3.2, -1.2, 0, 0, 0, 0, 0, 0, 0]))
    nodeId2 = dg.addNode(np.array([5.1, 3.2, -1.2, 0, 0, 0, 0, 0, 0, 0]))
    dg.addEdge(node_a=nodeId1, node_b=nodeId2)
    nodeId3 = dg.addNode(np.array([-3.1, 1.4, -2, 0, -1, 0, 0, 0, 0, 0]))
    dg.findAndSetConnections(nodeId3)
    dg.plot()
    dg.removeNode(nodeId2)
    dg.plot()
    #print(dg)


