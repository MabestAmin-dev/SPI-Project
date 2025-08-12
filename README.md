# Autonomous Maze-Navigating Robot with Web Control and SPI Communication

This project implements a **maze-navigating autonomous robot** with:
- Real-time **web-based control and visualization** (Flask + Socket.IO)
- **SPI communication** between a Raspberry Pi and ATmega microcontrollers
- **Sensor integration** (distance, gyro, line detection)
- **Navigation logic** (DFS/BFS style exploration with maze graph building)
- **Hardware control** for motors and a claw mechanism
- **Low-level C firmware** for SPI slaves and PD motor control

---

## 📜 Overview

### System Architecture
┌──────────┐ ┌──────────────────┐ ┌───────────────┐
│ Web UI │◄──────►│ Flask Webserver │◄──────►│ Navigation │
│ (Laptop) │ │ (Raspberry Pi) │ │ Logic │
└──────────┘ └──────────────────┘ └─────┬─────────┘
│
┌────────────┴────────────┐
│ SPI Communication │
│ (spidev, Raspberry Pi) │
└────────────┬────────────┘
│
┌──────────────────────────────────┼──────────────────────────┐
│ │ │
┌───────┴───────┐ ┌────────┴────────┐ ┌────────┴─────────┐
│ Motor Driver │ │ Sensor Reader │ │ Claw Controller │
│ ATmega C code │ │ ATmega C code │ │ ATmega C code │
└───────────────┘ └─────────────────┘ └──────────────────┘

---

## 📂 File Structure

### Python (High-Level Control & Web Interface)
| File | Purpose |
|------|---------|
| `main.py` | Flask web server + navigation process for basic simulation/demo. |
| `webserver.py` | Advanced web server + navigation system with real sensors. |
| `classes.py` | `Node`, `Maze`, and `Robot` classes for representing navigation state. |
| `SPI.py` | SPI test utility for sending/receiving data to/from ATmega devices. |

### C (Low-Level Firmware for ATmega)
| File | Purpose |
|------|---------|
| `SPI.c` | SPI slave firmware handling different responses based on incoming commands. |
| `spiSensor.c` | SPI slave firmware returning sensor data to the Raspberry Pi. |
| `reg.c` | PD controller implementation for wall-following / path correction. |

---

## ⚙️ Features

### Web Control & Visualization
- **Live maze rendering** in the browser.
- **Manual drive mode** for remote control.
- **Autonomous mode** for DFS/BFS exploration.
- **Live sensor data streaming** over WebSockets.

### Navigation Logic
- Maintains a **graph representation** of the maze.
- Detects and adds new nodes when exploring.
- Chooses next movement based on unexplored nodes stack.
- Detects tape markers for object pickup/drop-off.

### SPI Communication
- Raspberry Pi as **SPI master**.
- Multiple ATmega devices as **SPI slaves** (motors, sensors, claw).
- Exchange of motor commands and sensor data in real time.

### PD Motor Control
- ATmega PD controller for precise motor adjustments using front/back side sensors.

---

## 🚀 How to Run

### 1️⃣ Hardware Requirements
- Raspberry Pi (running Python code)
- ATmega microcontrollers for:
  - Motor control
  - Sensor reading
  - Claw control
- Sensors: IR distance sensors, gyro, line detector
- Motors + motor driver board
- VGA or LED display optional for debugging

### 2️⃣ Python Setup (Raspberry Pi)
```bash
sudo apt install python3-flask python3-socketio python3-spidev
pip install flask flask-socketio numpy
