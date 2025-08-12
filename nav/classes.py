import numpy as np
import datetime


class Node:
    def __init__(self, name, x=None, y=None,
                 neighbors=[None, None, None, None]) -> None:
        self.name = name
        self.neighbors = neighbors
        self.contains_robot = False
        self.visited = False
        self.x = x
        self.y = y


class Robot:
    def __init__(self, currentNode, maze, stack) -> None:
        # self.node = node
        self.done_moving = True

        # Navigation data
        self.done_moving = True
        self.currentNode = 'startNode'
        self.maze = maze
        self.stack = stack
        self.direction = "up"
        self.visited = ["startNode"]
        self.counter = 1
        self.targetNode = None
        self.nextNode = None
        self.forwardDistance = 0

        self.currentPartCommand = None
        self.newNeighbors = None
        self.forwardTimer = None

        # Stored data from sensors
        self.distance = 0  # stored in mm
        self.gyroTimestamps = []
        self.gyroValues = []
        self.distance_turned = 0
        self.tempDistance = 0
        self.iterObject = False
        self.distanceToMove = 77

        self.sideWalls = True

    def turnIntegral(self):

        time_diff = np.diff([t.astype(int) for t in self.gyroTimestamps])
        time_diff_seconds = time_diff / 1e6  # Convert nanoseconds to seconds
        # print(time_diff_seconds)

        if len(self.gyroValues) == 0 or len(time_diff_seconds) == 0:
            return 0
        return time_diff_seconds[-1] * self.gyroValues[-1]

    def addGyroData(self, sensor_data):

        # Get the current datetime
        current_time = datetime.datetime.now()

        # Convert the current datetime to np.datetime64
        current_time_np = np.datetime64(current_time)

        self.gyroTimestamps.append(current_time_np)
        self.gyroTimestamps.append(sensor_data[6])

    def resetData(self):
        self.gyroTimestamps = []
        self.gyroValues = []
        self.robotDistance = 0
        self.distance_turned = 0
        self.forwardDistance = 0
        self.distanceToMove = 77

    def addDistance(self):
        self.distance += 50

    '''
        self.x = 0
        self.y = 0
        self.old_x = 0
        self.old_y = 0

        self.direction = "up"
        self.speed = 0.4  # IRL measured value

    def update_position(self, elapsed_time):
        if self.direction == "right":
            self.x += self.speed * elapsed_time
        elif self.direction == "left":
            self.x -= self.speed * elapsed_time
        elif self.direction == "up":
            self.y -= self.speed * elapsed_time
        elif self.direction == "down":
            self.y += self.speed * elapsed_time

    def is_at_center(self):
        if abs(self.x - self.old_x) >= 1 or abs(self.y - self.old_y) >= 1:
            self.done_moving = True
            self.old_x = self.x
            self.old_y = self.y

    '''


class Maze:
    def __init__(self) -> None:
        # self.graph = dict()
        self.graph["startNode"] = ["", "", None, None]
        # self.graph["startNode"] = Node("startNode", 0, 0, ["", "", None, None])  # noqa
        self.array = [([None] * 25) for _ in range(25)]
