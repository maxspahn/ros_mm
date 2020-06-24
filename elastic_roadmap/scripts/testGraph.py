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
from obstacleArray import genSimpleObstacles, genDefaultObstacles, genEmptyObstacles

def blockPrint():
    sys.stdout = open(os.devnull, 'w')

def enablePrint():
    sys.stdout = sys.__stdout__

if __name__ == "__main__":
    defParams = ParametersElasticMap()
    defParams._minDistBase = 0.5
    defParams._maxTime = 5
    defParams._dt = 0.5
    eCon = MPCConnectorMM("MPCConnector", params=defParams, obstacles=genEmptyObstacles());
    #eCon = EuclideanConnector("EucCon", params=defParams);
    dg = DynamicGraph(eCon, params=defParams)
    config = dg.createSample()
    nodeId0 = dg.addNode(config)
    """
    config1 = np.copy(config)
    config2 = np.copy(config)
    config3 = np.copy(config)
    config1[0] += 1
    config2[1] += 1
    config3[0] += 2
    config3[6] += 0.2
    nodeId1 = dg.addNode(config1)
    dg.findAndSetConnections(nodeId1)
    nodeId2 = dg.addNode(config2)
    dg.findAndSetConnections(nodeId2)
    nodeId3 = dg.addNode(config3)
    dg.findAndSetConnections(nodeId3)
    config4 = np.copy(config2)
    config5 = np.copy(config2)
    config4[0] -= 1
    config5[0] += 1
    nodeId5 = dg.addNode(config5)
    dg.findAndSetConnections(nodeId5)
    nodeId4 = dg.addNode(config4)
    dg.findAndSetConnections(nodeId4)
    """
    for i in range(40):
        newConfig = dg.createSample(baseMus=config)
        if newConfig is not None:
            config = newConfig
            #print("Generate confiig and find connections for config : ", config)
            nodeId = dg.addNode(config)
            dg.findAndSetConnections(nodeId)
    print(dg)
    dg.plot(short=False)
    dg.saveGraph('../savedMaps/square_center_4')


