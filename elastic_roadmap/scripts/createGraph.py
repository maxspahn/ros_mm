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
    defParams._minDistBase = 3
    defParams._maxTime = 60
    defParams._dt = 0.4
    eCon = MPCConnectorMM("MPCConnector", params=defParams, obstacles=genDefaultObstacles());
    #eCon = EuclideanConnector("EucCon", params=defParams);
    dg = DynamicGraph(eCon, params=defParams)
    initConfig = dg._robotModel.getMeans()
    config = dg.createSample()
    nodeId0 = dg.addNode(config)
    for i in range(10):
        print(i)
        newConfig = dg.createSample(baseMus=initConfig)
        if newConfig is not None:
            config = newConfig
            #print("Generate confiig and find connections for config : ", config)
            nodeId = dg.addNode(config)
            dg.findAndSetConnections(nodeId)
    dg.plot(short=False)
    for i in range(10):
        print(i)
        newConfig = dg.createSample(baseMus=initConfig)
        if newConfig is not None:
            config = newConfig
            #print("Generate confiig and find connections for config : ", config)
            nodeId = dg.addNode(config)
            dg.findAndSetConnections(nodeId)
    dg.plot(short=False)
    for i in range(40):
        print(i)
        newConfig = dg.createSample(baseMus=initConfig)
        if newConfig is not None:
            config = newConfig
            #print("Generate confiig and find connections for config : ", config)
            nodeId = dg.addNode(config)
            dg.findAndSetConnections(nodeId)
    dg.plot(short=False)
    for i in range(50):
        print(i)
        newConfig = dg.createSample(baseMus=initConfig)
        if newConfig is not None:
            config = newConfig
            #print("Generate confiig and find connections for config : ", config)
            nodeId = dg.addNode(config)
            dg.findAndSetConnections(nodeId)
    dg.plot(short=False)
    dg.saveGraph('tempGraph')


