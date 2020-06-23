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
from params import ParametersElasticMap
from obstacleArray import genSimpleObstacles, genDefaultObstacles

def blockPrint():
    sys.stdout = open(os.devnull, 'w')

def enablePrint():
    sys.stdout = sys.__stdout__

if __name__ == "__main__":
    defParams = ParametersElasticMap()
    eCon = MPCConnectorMM("MPCConnector", params=defParams, obstacles=genSimpleObstacles());
    #eCon = EuclideanConnector("EucCon");
    dg = DynamicGraph(eCon, params=defParams)
    config = dg.createSample()
    nodeId0 = dg.addNode(config)
    for i in range(10):
        config = dg.createSample(baseMus=config)
        print("Generate confiig and find connections for config : ", config)
        nodeId = dg.addNode(config)
        dg.findAndSetConnections(nodeId)
    dg.plot(short=False)
    dg.saveGraph('../savedMaps/testGraph')


