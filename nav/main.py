from flask import Flask, render_template
from flask_socketio import SocketIO
from multiprocessing import Process
import time
from multiprocessing import Queue
from random import randint, choice
from dfs import move_maze, maze_to_array, locate_nodes
from classes import Maze, Node, Robot
from copy import deepcopy
import spidev
import platform
# spi.open(0, 0) -> pin 12, kopplad till B

app = Flask(__name__)
socketio = SocketIO(app)

@app.route('/')
def index_page() -> str:
    """
    Renders the index page.

    Returns:
        str: The website.
    """
    return render_template('index.html')


@socketio.on('sensorData')
def handle_message() -> None:
    """
    Emits sensor and maze data to the laptop.
    """
    data = [0, 0, 0, 0, 0, 0, 0]
    maze = [([None] * 25) for _ in range(25)]
    graph = dict()

    # Get latest sensor data
    while sensor_queue.qsize() != 0:
        data = sensor_queue.get()

    # Get latest maze data
    while maze_queue.qsize() != 0:
        graph = maze_queue.get()

    # Checks if there is a new maze to render
    if graph:
        maze = locate_nodes(graph)
        maze = move_maze(maze)
        maze = maze_to_array(maze)

    socketio.emit('sensorData', {"sensorData": data, "mazeData": maze})


@socketio.on('laptop_command')
def handle_command(command) -> None:
    """
    Handles manual commands from website
    """
    decision_queue.put(command)


def communication_system(
        decision_queue: Queue,
        sensor_queue: Queue,
        maze_queue: Queue
        ) -> None:
    """
    Starts the webserver.

    Args:
        decision_queue (Queue): Queue for manual control commands.
        sensor_queue (Queue): Queue for sensor data.
        maze_queue (Queue): Queue for maze data.
    """
    socketio.run(app, host="0.0.0.0", port=5000)


def read_sensor_data():
    data = [
        randint(1, 10),
        randint(1, 10),
        randint(1, 10),
        randint(1, 10),
        randint(1, 10),
        randint(1, 10),
        randint(1, 10)
    ]

    return data


def get_maze_data(maze: Maze) -> Maze:
    graph = {
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

    if choice([True, False]):
        if "node12" in graph.keys():
            graph.pop("node12")
            graph["node11"] = ["", "", "", "node10", (None, None)]
    else:
        graph["node12"] = ["", "", "", "node11", (None, None)]

    maze.graph = graph

    return maze


def is_at_center(sensor_data) -> bool:
    """
    Checks whether the robot has reaches its target.

    Args:
        sensor_data (_type_): _description_

    Returns:
        bool: _description_
    """
    return False


def update_robot(robot: Robot, elapsed_time) -> Robot:
    robot.update_position(elapsed_time)
    robot.is_at_center()
    # print(elapsed_time)
    return robot


def navigation_control(manual_control, command, robot: Robot, sensor_data):
    if not manual_control:
        # Not done
        if robot.done_moving:
            robot.done_moving = False
            print("running")
            return command
        else:
            return command
    else:
        return command


def open_claw(spi):
    spi.open(0, 1)
    spi.xfer([0x05])
    spi.close()


def close_claw(spi):
    spi.open(0, 1)
    spi.xfer([0x06])
    spi.close()


# def calibrate_light_sensor(spi):
#     spi.open(0, 0)

# Bjorn1234!
def send_decision_data(spi, command, manual_command):
    if manual_command:
        spi.open(0, 1)
        if command == "turn_right":
            spi.xfer([0x01])
        elif command == "turn_left":
            spi.xfer([0x02])
        elif command == "forward":
            spi.xfer([0x03])
        elif command == "reverse":
            spi.xfer([0x04])
        elif command == "stop":
            spi.xfer([0x00])
        # elif command == "claw_open":
        #     print("here")
        #     spi.xfer([0x05])
        # elif command == "claw_close":
        #     spi.xfer([0x06])
        spi.close()


def send_communication_data():
    pass


def navigation_system(
        decision_queue: Queue,
        sensor_queue: Queue,
        maze_queue: Queue
        ) -> None:
    """
    Main loop of the navigation system.

    Args:
        decision_queue (Queue): Queue for sharing laptop commands
        sensor_queue (Queue): Queue for sharing sensor data
        maze_queue (Queue): Queue for sharing maze data
    """
    manual_control: bool = False
    exploring: bool = True  # noqa
    decision_string = "n"
    command = "n"
    maze = Maze()
    robot = Robot(maze.graph["startNode"])

    startTime = time.time()

    firstLoop = True
    loopTime = time.time()

    if platform.machine() == "armv7l":
        # TODO: Overloada spidev konstruktorn så vi får två olika spi variabler
        spi = spidev.SpiDev()
        spi.open(0, 1)
        spi.max_speed_hz = 156250
        spi.mode = 0

    while True:
        sensor_data = read_sensor_data() 

        # Get communication data
        # For some reason queue doesn't work when passed to other function
        if decision_queue.qsize() != 0:
            decision_string = decision_queue.get()
            if decision_string == "checked":
                manual_control = True
            elif decision_string == "unchecked":
                manual_control = False
            elif decision_string == "claw_open":
                if platform.machine() == "armv7l":
                    open_claw(spi)
            elif decision_string == "claw_close":
                if platform.machine() == "armv7l":
                    close_claw(spi)
            else:
                command = decision_string
            print(command)

        if firstLoop:
            elapsed_time = time.time() - startTime
            firstLoop = False
            loopTime = time.time()
        else:
            elapsed_time = time.time() - loopTime
            loopTime = time.time()

        robot = update_robot(robot, elapsed_time)

        command = navigation_control(
            manual_control,
            command,
            robot,
            sensor_data
        )

        maze = get_maze_data(maze)
        if platform.machine() == "armv7l":
            send_decision_data(spi, command, manual_control)

        # Send communication data
        sensor_queue.put(sensor_data)
        maze_queue.put(maze.graph)
        time.sleep(0.1)


if __name__ == '__main__':
    # Starting the communication and navigation system in two different threads
    decision_queue = Queue()
    sensor_queue = Queue()
    maze_queue = Queue()
    navigation = Process(
        target=navigation_system,
        args=(decision_queue, sensor_queue, maze_queue)
    )
    communication = Process(
        target=communication_system,
        args=(decision_queue, sensor_queue, maze_queue)
    )

    communication.start()
    navigation.start()

    communication.join()
    navigation.join()

    print("Done")
