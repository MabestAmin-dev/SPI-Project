import spidev
import time

# Create an SPI object
spi = spidev.SpiDev()
spi.open(0,0)

# Set SPI speed and mode
spi.max_speed_hz = 156250
spi.mode = 0

# Function to send data to a specific ATmega1284P slave
def send_data_to_slave(select, data):
    # Select the appropriate slave by setting the SS pin
    if select == 0:
        print("HeLLO BABY!, device 0!")
        received_data = spi.xfer([data])
        print(received_data) # Send data to Select 1 (0x01)
    elif select == 1:
        print("HeLLO BABY!!!!!!!!!!!!!!!!!!!, device 1")
        spi.xfer([data])  # Send data to Select 2 (0x02)
    # You can add more cases for additional slaves as needed

def receiveddata():
    received_data = spi.readbytes(1)
    print(received_data)

# Example usage
try:
    while True:
        # Send data to Select 1
        spi.open(0, 0)  # Bus 0, Device 0
        send_data_to_slave(0, 0x10)  # Send 0x55 to Select 1


except KeyboardInterrupt:
    spi.close()
