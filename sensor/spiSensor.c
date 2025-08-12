#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define SPI_DDR DDRB
#define SPI_PORT PORTB
#define DD_SS DDB4
#define DD_MOSI DDB5
#define DD_MISO DDB6
#define DD_SCK DDB7



// Global variable to store data to be sent
volatile uint8_t dataToSend = 0;

// Global variable to store received data
volatile uint8_t receivedData = 0;

void SPI_init_slave() {
	// Set SS, MISO, and SCK as input
	SPI_DDR &= ~((1 << DD_SS) | (1 << DD_MISO) | (1 << DD_SCK));
	SPI_DDR = (1 << DD_MISO);

	// Enable SPI, SPI interrupt, set as a slave, and set SCLK freq to f/16
	SPCR = (1 << SPE) | (1 << SPIE);
}


uint8_t SPI_receive() {

	return SPDR;
}

void SPI_send(uint8_t data) {
	SPDR = data;
}

void processReceivedData(uint8_t data) {
	// Process the received data as needed
	// For example, you can store it in the receivedData variable
	receivedData = data;
}

ISR(SPI_STC_vect) {
	uint8_t received = SPI_receive();
	processReceivedData(received);
}

int main() {
	// Initialize SPI slave
	SPI_init_slave();
	sei();
	
	// Main loop
	while (1) {
		dataToSend = 0x30;
		SPI_send(dataToSend);
	}
	return 0;
}