#!/usr/bin/env python

import os
import sys
import pickle

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "./"))

import networkx as nx
import numpy as np
import matplotlib.pyplot as plt

from EuclideanConnector import EuclideanConnector
from robotModel import RobotModel
from params import ParametersElasticMap


class DynamicGraph(object):
    def __init__(
        self, connector, params=ParametersElasticMap(), robotModel=RobotModel("mm")
    ):
        self._variancesBase = params._variancesBase
        self._dim = params._dim
        self._nxGraph = nx.Graph()
        self._connector = connector
        self._index = 0
        self._maxTime = params._maxTime
        self._minDistBase = params._minDistBase
        self._robotModel = robotModel

    def hasTooCloseNeighbor(self, config):
        for node in self._nxGraph.nodes:
            if (
                np.linalg.norm(self._nxGraph.nodes[node]["config"][0:2] - config[0:2])
                < self._minDistBase
            ):
                return True
        return False

    def isValidConfig(self, config):
        return self._robotModel.isValidConfig(config) and not (
            self.hasTooCloseNeighbor(config)
        )

    def createSample(self, baseMus=[0, 0, 0]):
        mus = self._robotModel.getMeans()
        sigmas = self._robotModel.getSigmas()
        for i in range(20):
            qs = np.array([])
            for mu, sigma in zip(baseMus, self._variancesBase):
                qs = np.append(qs, np.random.normal(mu, sigma, 1)[0])
            for mu, sigma in zip(mus[3:10], sigmas[3:10]):
                qs = np.append(qs, np.random.normal(mu, sigma, 1)[0])
            if self.isValidConfig(qs):
                return qs
        return None

    def findAndSetConnections(self, node, maxConnections=5, maxDistBase=5):
        connections = 0
        for candidate in self._nxGraph.nodes:
            if candidate == node:
                continue
            if (
                np.linalg.norm(
                    self._nxGraph.nodes[candidate]["config"][0:2]
                    - self._nxGraph.nodes[node]["config"][0:2]
                )
                > maxDistBase
            ):
                continue
            dist = self.computeDistance(node, candidate)
            if dist != -1:
                self.addEdge(node_a=node, node_b=candidate, dist=dist)
                connections += 1
                if connections == maxConnections:
                    return

    def computeDistance(self, node_a, node_b):
        config_a = self._nxGraph.nodes[node_a]["config"]
        config_b = self._nxGraph.nodes[node_b]["config"]
        return self._connector.dist(config_a, config_b)

    def addNode(self, config):
        self._nxGraph.add_node(self._index, config=config)
        self._index += 1
        return self._index - 1

    def removeNode(self, node):
        self._nxGraph.remove_node(node)

    def addEdge(self, **kwargs):
        node_a = kwargs["node_a"]
        node_b = kwargs["node_b"]
        if "dist" in kwargs.keys():
            dist = kwargs["dist"]
        else:
            dist = self.computeDistance(node_a, node_b)
        if dist == -1:
            print("No connection possible")
        else:
            self._nxGraph.add_edge(node_a, node_b, weight=dist)

    def removeEdge(self, node_a, node_b):
        self._nxGraph.remove_edge(node_a, node_b)

    def cleanGraph(self):
        rmNodes = []
        for node in self._nxGraph.nodes:
            print(list(self._nxGraph.neighbors(node)))
            if len(list(self._nxGraph.neighbors(node))) == 0:
                rmNodes.append(node)
        for rmNode in rmNodes:
            self.removeNode(rmNode)

    def getPositionBase(self):
        basePos = [None] * 100
        for i in self._nxGraph.nodes:
            jointPos = self._nxGraph.nodes[i]["config"]
            basePos[i] = jointPos[0:2]
        return basePos

    def getConfig(self, node):
        return self._nxGraph.nodes[node]["config"]

    def findPath(self, node_a, node_b):
        return nx.multi_source_dijkstra(self._nxGraph, {node_a}, node_b)

    def plot(self, short=True):
        plt.plot()
        pos = self.getPositionBase()
        nx.draw(self._nxGraph, pos, with_labels=True, font_weight="bold")
        labels = nx.get_edge_attributes(self._nxGraph, "weight")
        formLabels = {
            (key1, key2): "{:.1f}".format(value)
            for (key1, key2), value in labels.items()
        }
        nx.draw_networkx_edge_labels(self._nxGraph, pos, edge_labels=formLabels)
        if short:
            plt.show(block=False)
            plt.pause(2)
            plt.close()
        else:
            plt.show()

    def __str__(self):
        description = "Dynamic Graph with Connector : " + self._connector.name + "\n"
        for i in self._nxGraph.nodes:
            description += str(i) + " : "
            for jointPos in self._nxGraph.nodes[i]["config"]:
                description += "{:.2f}".format(jointPos) + ", "
            description += "\n"
        for i in self._nxGraph.edges:
            description += (
                str(i)
                + " : "
                + "{:.2f}".format(self._nxGraph.edges[i]["weight"])
                + "\n"
            )
        return description

    def saveGraph(self, fileName):
        with open(fileName, "wb") as outputFile:
            pickle.dump(self, outputFile, pickle.HIGHEST_PROTOCOL)


if __name__ == "__main__":
    eCon = EuclideanConnector("simpleEuclideanConnector")
    mmModel = RobotModel("mm")
    dg = DynamicGraph(3, eCon, mmModel)
    print(dg.createSample())
    nodeId1 = dg.addNode(np.array([1.2, 3.2, -1.2]))
    nodeId2 = dg.addNode(np.array([5.1, 3.2, -1.2]))
    dg.addEdge(node_a=nodeId1, node_b=nodeId2)
    nodeId3 = dg.addNode(np.array([-3.1, 1.4, -2]))
    dg.findAndSetConnections(nodeId3)
    print(dg)
    dg.plot()
    dg.removeNode(nodeId2)
    dg.plot()
    print(dg)
    dg.saveGraph("testSave")
