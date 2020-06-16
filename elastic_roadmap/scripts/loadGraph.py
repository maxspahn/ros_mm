import os
import sys

sys.path.append(
    os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "../src/"
    )
)

from elastic_map import DynamicGraph
from mapLoader import DynamicGraphLoader

if __name__ == "__main__":
    dgl = DynamicGraphLoader('../savedMaps/testGraph')
    dg = dgl.loadDynamicGraph()
    dg.plot(short=False)
