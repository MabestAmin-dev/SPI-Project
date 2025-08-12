from flask import Flask, render_template
from flask_socketio import SocketIO
from multiprocessing import Process, Pipe


app = Flask(__name__)
socketio = SocketIO(app)


def send_data(child_connection):
    child_connection.send((manual_control, direction))


@app.route('/')
def index_page():
    return render_template('index.html')


@socketio.on('message')
def handle_message(data):
    # print('received message: ' + data)
    socketio.emit('message', data)
    print(data)


@socketio.on('sensorData')
def handle_message():
    data = [1, 2, 3, 4, 5, 6, 7]
    socketio.emit('sensorData', data)


@socketio.on('control_command')
def handle_command(command):
    direction = command
    print(direction)


def main():
    global direction
    global manual_control

    direction: str = 'n'
    manual_control: bool = False
    socketio.run(app, host="0.0.0.0", port=5000)


if __name__ == '__main__':
    main()
