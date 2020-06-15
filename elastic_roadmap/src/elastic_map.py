#!/usr/bin/env python

import os
import sys

sys.path.append(
    os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "./"
    )
)

import networkx as nx
import numpy as np
import matplotlib.pyplot as plt

from EuclideanConnector import EuclideanConnector
from robotModel import RobotModel


class DynamicGraph(object):

    def __init__(self, dim, connector, robotModel=RobotModel("mm"), maxDist=5.0):
        self.dim = dim
        self.nxGraph = nx.Graph()
        self.connector = connector
        self.index = 0;
        self.maxDist = maxDist
        self.robotModel = robotModel

    def addNode(self, config):
        self.nxGraph.add_node(self.index, config=config)
        self.index += 1
        return self.index - 1

    def isValidConfig(self, config):
        return self.robotModel.isValidConfig(config)

    def removeNode(self, node):
        self.nxGraph.remove_node(node)

    def findAndSetConnections(self, node):
        for candidate in self.nxGraph.nodes:
            if candidate == node:
                continue
            dist = self.computeDistance(node, candidate)
            if dist < self.maxDist:
                self.addEdge(node_a=node, node_b=candidate, dist=dist)

    def computeDistance(self, node_a, node_b):
        config_a = self.nxGraph.nodes[node_a]['config']
        config_b = self.nxGraph.nodes[node_b]['config']
        return self.connector.dist(config_a, config_b)

    def createSample(self, baseMus=[0, 0, 0], baseSigmas=[0.5, 0.5, 0.3]):
        mus = self.robotModel.getMeans()
        sigmas = self.robotModel.getSigmas()
        noValidConfigFound = True
        while noValidConfigFound:
            qs = np.array([])
            for mu, sigma in zip(baseMus, baseSigmas):
                qs = np.append(qs, np.random.normal(mu, sigma, 1)[0])
            for mu, sigma in zip(mus, sigmas):
                qs = np.append(qs, np.random.normal(mu, sigma, 1)[0])
            noValidConfigFound = not(self.isValidConfig(qs))
        return qs

    def addEdge(self, **kwargs):
        node_a = kwargs['node_a']
        node_b = kwargs['node_b']
        if 'dist' in kwargs.keys():
            dist = kwargs['dist']
        else:
            dist = self.computeDistance(node_a, node_b)
        if dist == -1:
            print("No connection possible")
        else:
            self.nxGraph.add_edge(node_a, node_b, weight=dist)

    def removeEdge(self, node_a, node_b):
        self.nxGraph.remove_edge(node_a, node_b);

    def plot(self, short=True):
        plt.plot()
        nx.draw(self.nxGraph, with_labels=True, font_weight='bold')
        if short:
            plt.show(block=False)
            plt.pause(2)
            plt.close()
        else:
            plt.show()

    def __str__(self):
        description = "Dynamic Graph with Connector : " + self.connector.name + "\n"
        for i in self.nxGraph.nodes:
            description += str(i) + " : "
            for jointPos in self.nxGraph.nodes[i]['config']:
                description += '{:.2f}'.format(jointPos) + ', '
            description += '\n'
        for i in self.nxGraph.edges:
            description += str(i) + " : " + '{:.2f}'.format(self.nxGraph.edges[i]['weight']) + "\n"
        return(description)

if __name__ == "__main__":
    eCon = EuclideanConnector("simpleEuclideanConnector");
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


