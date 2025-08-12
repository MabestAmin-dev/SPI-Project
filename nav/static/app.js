var socket = io.connect('http://' + document.domain + ':' + location.port);

socket.on('message', function(data) {
    var chat = document.getElementById('chat');
    chat.innerHTML += data + '<br>';
});

let buttonChange = false;
const checkbox = document.getElementById('toggle');
const claw = document.getElementById('claw');

checkbox.addEventListener('change', e => {
    if(e.target.checked === true) {
        socket.emit('laptop_command', 'checked')
    }
    if(e.target.checked === false) {
        socket.emit('laptop_command', 'unchecked')
    }
});

claw.addEventListener('change', e => {
    if(e.target.checked === true) {
        socket.emit('laptop_command', 'claw_close')
    }
    if(e.target.checked === false) {
        socket.emit('laptop_command', 'claw_open')
    }
});

document.addEventListener("keypress", function(event) {
    console.log(event.keyCode);
    let manual_control_box = document.getElementById('toggle');
    if (manual_control_box.checked) {
        if (event.keyCode == 119) {
            socket.emit('laptop_command', 'forward');
        } else if (event.keyCode == 97) {
            socket.emit('laptop_command', 'turn_left');
        } else if (event.keyCode == 100) {
            socket.emit('laptop_command', 'turn_right');
        } else if (event.keyCode == 115) {
            socket.emit('laptop_command', 'reverse');
        } else if (event.keyCode == 120) {
            socket.emit('laptop_command', 'stop');
        }
    } 
});

socket.on('sensorData', function(data) {
    sensorData = data.sensorData;
    mazeData = data.mazeData;
    document.getElementById('sensor1').innerHTML = "Front Left : " +  sensorData[0];
    document.getElementById('sensor2').innerHTML = "Front Right: " +  sensorData[1];
    document.getElementById('sensor3').innerHTML = "Back Left  : " +  sensorData[2];
    document.getElementById('sensor4').innerHTML = "Back Right : " +  sensorData[3];
    document.getElementById('sensor5').innerHTML = "Front      : " +  sensorData[4];
    document.getElementById('sensor6').innerHTML = "Back       : " +  sensorData[5];
    document.getElementById('sensor7').innerHTML = "Gyro       : " +  sensorData[6];
    document.getElementById('sensor8').innerHTML = "Reflex     : " +  sensorData[7];

    var validMaze = false;
    for (var i = 0; i < 25; i++) {
        for (var j = 0; j < 25; j++) {
            if (mazeData[i][j]) {
                ctx.clearRect(0, 0, canvas.width, canvas.height);
                drawMatrix(mazeData);
                break;
            }
        }   
        if (validMaze) {
            break;
        }
    }
});

let manual_control_box = document.getElementById('toggle');
manual_control_box.checked = true;
socket.emit('laptop_command', 'checked')

// Main loop to get data
function infiniteLoop(delay) {
    function loop() {
        // Your code to be executed in the loop
        socket.emit('sensorData')
    

        // Schedule the next iteration
        setTimeout(loop, delay);
    }

    // Start the infinite loop
    loop();
}


function testMatrix() {
    var matrix = [];
    for (var i = 0; i < 25; i++) {
        matrix[i] = [];
        for (var j = 0; j < 25; j++) {
            matrix[i][j] = undefined;
        }
    }

    // matrix[12][12] = ["node", "", "", "node"];
    // matrix[12][13] = ["", "node", "", "node"];
    // matrix[13][12] = ["node", "", "node", ""];
    // matrix[13][13] = ["", "node", "node", ""];
    // matrix[16][16] = ["", "node", "node", "node"];

    return matrix;
}

function randomMaze(matrix) {
    matrix[Math.floor(Math.random() * 25)][Math.floor(Math.random() * 25)] = ["", "", "", ""];
    matrix[Math.floor(Math.random() * 25)][Math.floor(Math.random() * 25)] = undefined;

    return matrix;
}


var canvas = document.getElementById("myCanvas");
var ctx = canvas.getContext("2d");
var cellSize = 24;
var matrixScale = 1;

canvas.width = 25 * cellSize;
canvas.height = 25 * cellSize;

var colors = {
    undefinedColor: "gray",
    pathColor: "white",
    borderColor: "black"
};


function drawMatrix(matrix) {
    for (var i = 0; i < 25; i++) {
        for (var j = 0; j < 25; j++) {
            var x = j * cellSize;
            var y = i * cellSize;

            if (matrix[i][j]) {
                var [left, up, right, down] = matrix[i][j];
                ctx.fillStyle = colors.pathColor;
                ctx.fillRect(x, y, cellSize, cellSize);

                ctx.fillStyle = colors.borderColor;
                if (right === "") {
                    ctx.fillRect(x + cellSize - 2, y, 2, cellSize);
                }
                if (left === "") {
                    ctx.fillRect(x, y, 2, cellSize);
                }
                if (up === "") {
                    ctx.fillRect(x, y, cellSize, 2);
                }
                if (down === "") {
                    ctx.fillRect(x, y + cellSize * matrixScale - 2, cellSize * matrixScale, 2);
                }
            }
        }
    }
}

function submitInput(inputNumber) {
    // Get the input value based on the input number
    const inputValue = document.getElementById(`input${inputNumber}`).value;
    // Do something with the input value (e.g., display, process, etc.)
    //console.log(`Input ${inputNumber} value:`, 'p' + inputValue);
    // You can add further processing or actions here
    if (inputNumber == 1) {
        console.log(`Input ${inputNumber} value:`, 'p' + inputValue);
        socket.emit('laptop_command', 'p' + inputValue.toString());
    } else if (inputNumber == 2) {
        console.log(`Input ${inputNumber} value:`, 'd' + inputValue);
        socket.emit('laptop_command', 'd' + inputValue.toString());
    }

}

// Usage:
infiniteLoop(1000); // This will create an infinite loop with a 1-second delay between each iteration