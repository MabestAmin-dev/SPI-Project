from multiprocessing import Process, Queue, Pipe
from webserver import send_data
from webserver import manual_control, direction


def read_sensor_data():
    pass


def read_communication_data():
    parent_connection, child_connection = Pipe()
    p = Process(target=send_data, args=(child_connection,))
    p.start()
    print(parent_connection.recv())   # prints "Hello"


def send_communication_data():
    pass


def send_decision_data(decision):
    pass


def navigation_control(sensor_data, communication_data):
    pass


def navigation_system():
    while True:
        read_sensor_data()
        read_communication_data()
        navigation_control()
        send_decision_data()
        send_communication_data()


if __name__ == "__main__":
    print(manual_control, direction)
    read_communication_data()
