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
from obstacleArray import genSimpleObstacles, genDefaultObstacles

def blockPrint():
    sys.stdout = open(os.devnull, 'w')

def enablePrint():
    sys.stdout = sys.__stdout__

if __name__ == "__main__":
    eCon = MPCConnectorMM("MPCConnector", 15, maxDist = 8, obstacles=genSimpleObstacles());
    #eCon = EuclideanConnector("EucCon");
    dg = DynamicGraph(10, eCon, maxDist=50)
    config = dg.createSample()
    nodeId0 = dg.addNode(config)
    for i in range(10):
        print("Generate confiig and find connections")
        config = dg.createSample(baseMus=config)
        nodeId = dg.addNode(config)
        dg.findAndSetConnections(nodeId)
    print(dg)
    dg.plot()


