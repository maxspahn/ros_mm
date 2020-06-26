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
    if len(sys.argv) == 2:
        fileName = sys.argv[1]
    else:
        fileName = '../savedMaps/square_center_4'
    print(fileName)
    dgl = DynamicGraphLoader(fileName)
    dg = dgl.loadDynamicGraph()
    dg.plot(short=False)
    dg.cleanGraph()
    dg.plot(short=False)
    dg.saveGraph('tempGraph')
    """
    [length, path] = dg.findPath(49, 8)
    print(path)
    for state in path:
        goal = dg.getConfig(state)
        print(goal)
    dg.plot(short=False)
    """
