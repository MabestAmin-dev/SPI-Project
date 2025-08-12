from flask import Flask, render_template
from flask_socketio import SocketIO
from multiprocessing import Process
import time
from multiprocessing import Queue
from random import choice
from dfs import move_maze, maze_to_array, locate_nodes, next_node, shift_elements_right  # noqa
from classes import Robot
import spidev
import platform
from test_integral import turnIntegral
import datetime

import numpy as np

app = Flask(__name__)
socketio = SocketIO(app)


@app.route('/')
def index_page() -> str:
    """
    Renders the index page.

        # Convert the current datetime to np.datetime64
        current_time_np = np.datetime64(current_time)

        timestamps.append(current_time_np)

    Returns:
        str: The website.
    """
    return render_template('index.html')


@socketio.on('sensorData')
def handle_message() -> None:
    """
    Emits sensor and maze data to the laptop.
    """
    data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    maze = [([None] * 25) for _ in range(25)]
    graph = dict()

    # Get latest sensor data
    while sensor_queue.qsize() != 0:
        data = sensor_queue.get()

    # Get latest maze data
    while maze_queue.qsize() != 0:
        graph = maze_queue.get()


    for key in graph:
        
        if key == "startNode":
            graph[key].append((0, 0))
        else:
            graph[key].append((None, None))

    
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
    Starts the web server.

    Args:
        decision_queue (Queue): Queue for manual control commands.
        sensor_queue (Queue): Queue for sensor data.
        maze_queue (Queue): Queue for maze data.
    """
    socketio.run(app, host="0.0.0.0", port=5000)


def read_sensor_data(sensor_spi, sensor_data):
    # sensor[0] = front_left
    # sensor[1] = front_right
    # sensor[2] = back_left
    # sensor[3] = back_right
    # sensor[4] = front
    # sensor[5] = back
    # TODO: sensor[7] = REFLEX BOOLEAN

    sensors = [0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09]

    if sensor_spi:
        sensor_spi.open(0, 0)

        for i in range(10):
            _ = sensor_spi.xfer([sensors[i]])
            sensor_L = sensor_spi.xfer([sensors[i]])
            sensor_H = sensor_spi.xfer([sensors[i]])
            sensor_data[i] = sensor_H[0] * 16**2 + sensor_L[0]

        sensor_spi.close()

    sensor_data[6] = abs(sensor_data[6] - 506)
    
    if sensor_data[7] > 0 and sensor_data[7] < 5:
        sensor_data[7] = 1
    elif sensor_data[7] >= 5:
        sensor_data[7] = 2
    return sensor_data


def portion_movement_done(robot: Robot, sensor_data):
    if robot.currentPartCommand == "left":
        return robot.distance_turned >= 117
    elif robot.currentPartCommand == "right":
        return robot.distance_turned >= 117
    elif robot.currentPartCommand == "forward":
        if sensor_data[4] < 37:
            return sensor_data[4] < 17
             
        #elapsed_time = time.time() - robot.forwardTimer > 1.6
        else:
            return robot.forwardDistance > robot.distanceToMove
    elif robot.currentPartCommand == "backwards":
        return robot.distance_turned >= 240 # 234   

    return False


def get_neighbors(robot, sensor_data, sensor_spi):
    neighbors = ["", "", "", ""]
    sensors = [0, 4, 1, 5]

    front_values = []
    back_values = []
    left_values = []
    right_values = []
    
    sensor_value = []
    
    for i in range(75):
        sensor_value = read_sensor_data(sensor_spi, sensor_data)
        left_values.append((sensor_value[0] + sensor_value[2]) // 2)
        right_values.append((sensor_value[1] + sensor_value[3]) // 2)
        front_values.append(sensor_value[4])
        back_values.append(sensor_value[5])
        time.sleep(0.02)

    front_values.sort()
    back_values.sort()
    left_values.sort()
    right_values.sort()
    sensor_data[0] = left_values[len(left_values) // 2]
    sensor_data[1] = right_values[len(right_values) // 2]
    sensor_data[2] = front_values[len(front_values) // 2]
    sensor_data[3] = back_values[len(back_values) // 2]

    for i in range(4):

        value = sensor_data[sensors[i]]

        if value > 30 and neighbors[i] == "":
            neighbors[i] = True

    print(sensor_data)

    if robot.direction == "up":  # Up
        neighbors = shift_elements_right(neighbors, 0)
        neighbors[3] = robot.currentNode
    elif robot.direction == "left":  # Left
        neighbors = shift_elements_right(neighbors, 3)
        neighbors[2] = robot.currentNode
    elif robot.direction == "right":  # Right
        neighbors = shift_elements_right(neighbors, 1)
        neighbors[0] = robot.currentNode
    elif robot.direction == "down":  # Down
        neighbors = shift_elements_right(neighbors, 2)
        neighbors[1] = robot.currentNode

    temp = robot.currentNode
    robot.currentNode = robot.stack.pop()
    robot.visited.append(robot.currentNode)

    temp_list = []
    for i in range(0, len(neighbors)):

        if neighbors[i] != temp and neighbors[i] != '' and neighbors[i] not in robot.visited:
            robot.counter += 1
            # Create the rest of the neighbors. They are new
            neighbors[i] = str(robot.counter)
            temp_list.append(str(robot.counter))

    temp_list.reverse()
    robot.stack += temp_list

    robot.maze[robot.currentNode] = neighbors


def navigation_control(manual_control,
                       command,
                       robot: Robot,
                       sensor_data,
                       driver_spi,
                       sensor_spi):

    if not manual_control:

        # Store if node have new neighbors
        if robot.done_moving:
            send_decision_data(driver_spi, "stop", sensor_data)
            print("Done moving")
            time.sleep(2)
                
            # Done exploring put target node on stack
            if not robot.stack and robot.currentNode != robot.targetNode:
                print("We done")
                #robot.targetNode = "startNode"
                #robot.stack.append("startNode")
                robot.stack.append(robot.targetNode)

            # Done exploring and at target node

            if not robot.stack and robot.currentNode == robot.targetNode:
                # Pick up object
                print("super done")
                robot.stack.append("startNode")
                manual_control = True

            # Not done exploring
            command, newNode, nextNode = next_node(robot)

            robot.nextNode = nextNode
            robot.newNeighbors = newNode  # boolean
            # Set current Part command from dfs or bfs
            if robot.forwardTimer is None and command == "forward":
                robot.forwardTimer = time.time()

            robot.command = command
            robot.currentPartCommand = command
            robot.done_moving = False

            # Reset robot data for gyro and distance
            robot.resetData()
            if platform.machine() == "armv7l":
                send_decision_data(driver_spi, "stop", sensor_data)

        else:
            
            # if first tick without walls
            if robot.currentPartCommand == "forward":
                
                if robot.targetNode == None and sensor_data[7] == 1: # We found tape for first time 
                    robot.targetNode = nextNode # Next node is the target node 
                    
                    robot.tempDistance = robot.forwardDistance
                    robot.iterObject = True

                    robot.currentPartCommand = 'backwards'
                            
                                
                if robot.targetNode != None and sensor_data[7] == 2: # If we find tape and want
                    # to pick up object
                    robot.tempDistance = robot.forwardDistance
                    robot.iterObject = True
                    time.sleep(1)
                    close_claw(driver_spi, sensor_data)

                    robot.currentPartCommand = 'backwards'
                    


                if not sidewalls(sensor_data) and robot.sideWalls == True:
                    # robot.forwardTimer = time.time() - 0.8
                    robot.sideWall = False
                elif sidewalls(sensor_data):
                    robot.sideWalls = True
            

            if portion_movement_done(robot, sensor_data):
                if robot.currentPartCommand in ["left", "right", "backwards"]:
                    if robot.iterObject:
                        robot.iterObject = False
                        robot.distanceToMove = robot.tempDistance

                    robot.currentPartCommand = "forward"
                    robot.resetData()
                    robot.forwardTimer = time.time()

                # Done with command from dfs or bfs
                elif robot.currentPartCommand == "forward":
                    robot.forwardTimer = None

                    robot.done_moving = True
                    if platform.machine() == "armv7l":
                        send_decision_data(driver_spi, "stop", sensor_data)
                    # If at new node and need to add neighbors
                    if robot.newNeighbors:
                        get_neighbors(robot, sensor_data, sensor_spi)
                    else:
                        robot.currentNode = robot.nextNode
                        if robot.currentNode == robot.stack[-1]:
                            robot.stack.pop()

                        # update current node
                
                if platform.machine() == "armv7l":
                    send_decision_data(driver_spi, "stop", sensor_data)

            # If not done with portion movement
            else:
                # Update distance and gyro values
                #if robot.currentPartCommand in ["left", "right", "backwards"]:
                #    #robot.addGyroData(sensor_data)
                if robot.currentPartCommand == "forward":

                    if robot.forwardTimer is None:
                        robot.forwardTimer = time.time()

                    # robot.addDistance()

                if platform.machine() == "armv7l":
                    send_decision_data(
                        driver_spi, robot.currentPartCommand, sensor_data
                    )

    # If manuel controlled
    else:
        if platform.machine() == "armv7l":
            send_decision_data(driver_spi, command, sensor_data)
        return command
    
    return robot.currentPartCommand
    

def sidewalls(sensor_data):
    # If sensor 0 or 1 don't see a wall a tick it returns false
    if sensor_data[0] >= 30 or sensor_data[1] >= 30:
        return False
    return True


def open_claw(driver_spi, sensor_data):
    driver_spi.open(0, 1)
    driver_spi.xfer([0x05])
    send_sensor_data(driver_spi, sensor_data)
    driver_spi.close()


def close_claw(driver_spi, sensor_data):
    driver_spi.open(0, 1)
    driver_spi.xfer([0x06])
    send_sensor_data(driver_spi, sensor_data)
    driver_spi.close()


def change_p(driver_spi, p_value):
    driver_spi.open(0, 1)
    driver_spi.xfer([0x07])
    driver_spi.xfer([p_value & 0xFF])
    driver_spi.close()


def change_d(driver_spi, d_value):
    driver_spi.open(0, 1)
    driver_spi.xfer([0x08])
    driver_spi.xfer([d_value & 0xFF])
    driver_spi.close()


# def calibrate_light_sensor(spi):
#     spi.open(0, 0)


def convert_to_8bit_hex(number: int) -> tuple[int, int]:
    """
    Returns the high and low hex values up to 255

    Args:
        number (int): The number to convert

    Returns:
        tuple[int, int]: high, low
    """
    number &= 0xFFFF  # Mask the number to 16 bits

    high_byte = (number >> 8) & 0xFF  # Extract the high 8 bits
    low_byte = number & 0xFF  # Extract the low 8 bits

    return high_byte, low_byte


def send_sensor_data(driver_spi, sensor_data):
    # sensor[0] = front_left
    # sensor[1] = front_right
    # sensor[2] = back_left
    # sensor[3] = back_right
    # TODO: sensor[7] = REFLEX BOOLEAN

    for i in range(4):
        sensor = sensor_data[i]
        sensor_high, sensor_low = convert_to_8bit_hex(sensor)
        driver_spi.xfer([sensor_high])
        driver_spi.xfer([sensor_low])


def send_decision_data(driver_spi, command, sensor_data) -> None:
    driver_spi.open(0, 1)
    if command in ["turn_right", "right"]:
        driver_spi.xfer([0x01])
        send_sensor_data(driver_spi, sensor_data)
    elif command in ["turn_left", "left", "backwards"]:
        driver_spi.xfer([0x02])
        send_sensor_data(driver_spi, sensor_data)
    elif command == "forward":
        driver_spi.xfer([0x03])
        send_sensor_data(driver_spi, sensor_data)
    elif command == "reverse":
        driver_spi.xfer([0x04])
        send_sensor_data(driver_spi, sensor_data)
    elif command == "stop":
        driver_spi.xfer([0x00])
        send_sensor_data(driver_spi, sensor_data)
    driver_spi.close()


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

    manual_control: bool = True
    decision_string = "stop"
    command = "stop"
    sensor_data = [0 for _ in range(10)]

    stack = []
    maze = {}
    maze["startNode"] = ["", "1", "", ""]
    robot = Robot("startNode", maze, stack)

    driver_spi = None
    sensor_spi = None

    if platform.machine() == "armv7l":
        driver_spi = spidev.SpiDev(0, 1)
        driver_spi.max_speed_hz = 156250
        driver_spi.mode = 0

        sensor_spi = spidev.SpiDev(0, 0)
        sensor_spi.max_speed_hz = 156250
        sensor_spi.mode = 0
    else:
        sensor_spi = 0
        driver_spi = 0

    robot.stack.append('1')
    while True:
        sensor_data = read_sensor_data(sensor_spi, sensor_data)
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
                    open_claw(driver_spi, sensor_data)
            elif decision_string == "claw_close":
                if platform.machine() == "armv7l":
                    close_claw(driver_spi, sensor_data)
            elif decision_string[0:1] == "p":
                if platform.machine() == "armv7l":
                    change_p(
                        driver_spi,
                        int(decision_string[1:len(decision_string)])
                    )
            elif decision_string[0:1] == "d":
                if platform.machine() == "armv7l":
                    change_d(
                        driver_spi,
                        int(decision_string[1:len(decision_string)])
                    )
            else:
                command = decision_string
            # print(command)
        
        #if command == 'forward':
        robot.forwardDistance += sensor_data[8]
            #print(robot.forwardDistance)

        #if robot.forwardDistance > 75:
        #    command = "stop"
        #    robot.forwardDistance = 0
        #
        #if command == 'turn_left':
        #    robot.forwardDistance = 0

        if command in ["turn_left", "turn_right", "left", "right", "backwards"]:
            gyro_value = sensor_data[6]
            robot.gyroValues.append(gyro_value)

            current_time = datetime.datetime.now()

            # Convert the current datetime to np.datetime64
            current_time_np = np.datetime64(current_time)

            robot.gyroTimestamps.append(current_time_np)
            robot.distance_turned += robot.turnIntegral()
            # print(robot.distance_turned)


        command = navigation_control(
            manual_control,
            command,
            robot,
            sensor_data,
            driver_spi,
            sensor_spi
        )
        # Send communication data
        sensor_queue.put(sensor_data)
        maze_queue.put(robot.maze)
        time.sleep(0.02)


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
