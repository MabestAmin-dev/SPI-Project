#include <avr/io.h>
#include <avr/interrupt.h>

#define SPI_DDR DDRB
#define SPI_PORT PORTB
#define SS_PIN PB4

// Global variable to store data to be sent
volatile uint8_t dataToSend = 0;

// Global variable to store received data
volatile uint8_t receivedData = 0;

// Global variable to track the SS pin state
volatile uint8_t ssActive = 0;

void SPI_init_slave() {
    // Set SS, MISO, and SCK as input
    SPI_DDR = (1 << PB6);

    // Enable SPI, SPI interrupt, set as a slave, and set SCLK freq to f/16
    SPCR = (1 << SPE) | (1 << SPIE) | (1 << SPR0);

    // Set the SS pin to 1 initially
    // SPI_PORT |= (1 << SS_PIN);
}

uint8_t SPI_receive() {
    // Wait for data reception
    while (!(SPSR & (1 << SPIF)));

    // Return received data
    return SPDR;
}

void SPI_send(uint8_t data) {
    // Wait for the previous transfer to complete
    while (!(SPSR & (1 << SPIF)));

    // Send data
    SPDR = data;
}

void processReceivedData(uint8_t data) {
    // Process the received data as needed
    // For example, you can store it in the receivedData variable
    receivedData = data;
}

int main() {
    // Initialize SPI slave
    SPI_init_slave();

    // Main loop
    while (1) {
        if (ssActive) {
            // The SS pin is active, meaning data has been received
            uint8_t received = SPI_receive();
            processReceivedData(received);
            
            // Toggle the SS pin
            SPI_PORT |= (1 << SS_PIN);  // Set SS pin to 1
            ssActive = 0;
        } else {
            // Check the value of dataToSend and send data accordingly
            if (dataToSend == 1) {
                // Send data specific to Select 1
                SPI_send(0x11); // Modify this with the data you want to send
            } else if (dataToSend == 2) {
                // Send data specific to Select 2
                SPI_send(0x22); // Modify this with the data you want to send
            }
        }
    }
    return 0;
}

ISR(SPI_STC_vect) {
    // SPI data received, toggle the SS pin
    ssActive = !ssActive;
}
