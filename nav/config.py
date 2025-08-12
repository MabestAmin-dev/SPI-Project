# Nodes have neighbors represented as [right, left, up, down]
# Empty value represents a wall as neighbor
# Tuple coordinates represented as (y, x)
test_graph_1 = {
            "startNode": ["", "", "node1", "in", (0, 0)],
            "node1": ["node2", "node3", "", "startNode", (None, None)],
            "node2": ["node4", "node1", "", "", (None, None)],
            "node3": ["node1", "node5", "", "", (None, None)],
            "node4": ["", "node2", "", "", (None, None)],
            "node5": ["node3", "", "node6", "", (None, None)],
            "node6": ["node7", "", "", "node5", (None, None)],
            "node7": ["node8", "node6", "", "", (None, None)],
            "node8": ["node9", "node7", "", "", (None, None)],
            "node9": ["node10", "node8", "", "", (None, None)],
            "node10": ["", "node9", "node11", "", (None, None)],
            "node11": ["", "", "node12", "node10", (None, None)],
            "node12": ["", "", "", "node11", (None, None)]
}


