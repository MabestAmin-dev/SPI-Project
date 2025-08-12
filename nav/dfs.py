from collections import deque
from classes import Node, Robot, Maze
# Sample graph represented as an adjacency list
import time

# Finds the shortest path from the robot to a destination in the maze
def sbd(robot: Robot, destination: str) -> None:
    visited = set()
    queue = deque()
    queue.append((robot.currentNode, [robot.currentNode]))

    while queue:
        node, path = queue.popleft()

        if node not in visited:
            visited.add(node)

            if node == destination:

              
                return path[1]

            for neighbor in robot.maze[node]:
                if neighbor != '' and neighbor not in visited:
                    new_path = list(path)  # Create a new copy of the path
                    new_path.append(neighbor)
                    queue.append((neighbor, new_path))
         

    return None


# Shifts elelements an arbitrary number of times to the right
def shift_elements_right(lst, num_shifts):
    length = len(lst)

    # Handle cases where the list is empty or the number of shifts is 0
    if length == 0 or num_shifts == 0:
        return lst

    # Perform the shifts
    num_shifts = num_shifts % length  # Ensure num_shifts is within the range of the list length # noqa
    shifted_list = lst[-num_shifts:] + lst[:-num_shifts]

    return shifted_list


# Returns a command
def next_node(robot):
    direction = ["left", "forward", "right", "backwards"]
    translation = ["left", "up", "right", "down"]

    if robot.direction == "up":  # Up
        direction = shift_elements_right(direction, 0)
    elif robot.direction == "left":  # Left
        direction = shift_elements_right(direction, 3)
    elif robot.direction == "right":  # Right
        direction = shift_elements_right(direction, 1)
    elif robot.direction == "down":  # Down
        direction = shift_elements_right(direction, 2)

    for count, neighbor in enumerate(robot.maze[robot.currentNode]):

        if neighbor != "" and neighbor not in robot.visited:
            robot.direction = translation[count]
           # robot.stack.pop()
            return direction[count], True, None

    else:
    
        # BFS
        if not robot.stack:
            return None
        node = sbd(robot, robot.stack[-1])
        robot.direction = translation[
            robot.maze[robot.currentNode].index(node)
        ]

        #print("target", robot.stack[-1])
        #print("stack", robot.stack)
        #print("node", node)
        #if robot.stack[-1] == node:
        return direction[robot.maze[robot.currentNode].index(node)], False, node
        #else:
        #    return direction[robot.maze[robot.currentNode].index(node)], False, node


def locate_nodes_class(maze: Maze):
    visited = set()
    queue = deque()
    start = "startNode"
    queue.append(start)
    while queue:
        node_name = queue.popleft()
        node: Node = maze.graph[node_name]


        visited.add(node_name)
        x = node.x
        y = node.y
        i = 0
        for neighbor in node.neighbors:
            # print(neighbor)
            if neighbor not in visited and neighbor != '' and neighbor != 'in' and neighbor:
                if i == 0:
                    maze.graph[neighbor].x = x + 1
                    maze.graph[neighbor].y = y
                    # maze.graph[neighbor][4] = (y, x + 1)
                elif i == 1:
                    maze.graph[neighbor].x = x - 1
                    maze.graph[neighbor].y = y
                    # maze[neighbor][4] = (y, x - 1)
                elif i == 2:
                    maze.graph[neighbor].x = x
                    maze.graph[neighbor].y = y - 1
                    # maze[neighbor][4] = (y - 1, x)
                elif i == 3:
                    maze.graph[neighbor].x = x
                    maze.graph[neighbor].y = y + 1
                    # maze[neighbor][4] = (y + 1, x)

                queue.append(neighbor)
                # print(queue)
            i += 1

    return maze


def locate_nodes(maze):
    visited = set()
    queue = deque()
    start = 'startNode'
    queue.append((start))
    while queue:
        node = queue.popleft()
        # print(node)

        # [right, left, up, down]
        # [left, up, right, down]
        # [left, up, right, down, (x, y)]

        visited.add(node)
        coords = maze[node][4]
        i = 0
        for neighbor in maze[node][0:-1]:

            # print("maze data", maze[neighbor][4])
            # print(neighbor)
            if neighbor not in visited and neighbor != '' and neighbor != 'in' and neighbor in maze:
                if i == 0:
                    maze[neighbor][4] = (coords[0], coords[1] - 1)
                elif i == 1:
                    maze[neighbor][4] = (coords[0] - 1, coords[1])
                elif i == 2:
                    maze[neighbor][4] = (coords[0], coords[1] + 1)
                elif i == 3:
                    maze[neighbor][4] = (coords[0] + 1, coords[1])

                queue.append(neighbor)
                # print(queue)
            i += 1

    return maze


def move_maze(maze: dict[str, list]) -> dict[str, list]:
    min_x = float('inf')
    min_y = 0
    max_x = 0
    max_y = 0
    for key, value in maze.items():
        y = value[4][0]
        x = value[4][1]
        if x > max_x:
            max_x = x
        if x < min_x:
            min_x = x
        if y < min_y:
            min_y = y

    middle_y = int((min_y + max_y) / 2)
    middle_x = int((min_x + max_x) / 2)

    for value in maze.values():
        value[4] = (value[4][0] - middle_y + 12, value[4][1] - middle_x + 12)
    # print(maze)
    return maze


def maze_to_array(maze: dict[str, list]) -> list[list]:
    array_maze = [([None] * 25) for _ in range(25)]

    for value in maze.values():
        # print(maze)
        coord_y = value[4][0]
        coord_x = value[4][1]
        array_maze[coord_y][coord_x] = value[0:4]
    return array_maze


if __name__ == "__main__":
    visited = set()

    # maze = locate_nodes()
    # maze = move_maze(maze)
    # maze = maze_to_array(maze)
    # print(maze)
