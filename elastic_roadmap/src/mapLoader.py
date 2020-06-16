import pickle
from elastic_map import DynamicGraph

class DynamicGraphLoader(object):

    def __init__(self, fileName):
        self._fileName = fileName

    def loadDynamicGraph(self):
        with open(self._fileName, 'rb') as inputFile:
            return pickle.load(inputFile)


if __name__ == "__main__":
    dgl = DynamicGraphLoader('../savedMaps/testSave')
    dg = dgl.loadDynamicGraph()
    print(dg)
