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

volatile int counter = 1;

void SPI_init_slave() {
	// Set SS, MISO, and SCK as input
	SPI_DDR &= ~((1 << DD_SS) | (1 << DD_MOSI) | (1 << DD_SCK));
	SPI_DDR = (1 << DD_MISO);

	// Enable SPI, SPI interrupt, set as a slave, and set SCLK freq to f/16
	SPCR = (1 << SPE) | (1 << SPIE);
}

void setup_PWM() {
	// Set PD5 (OC1A) pin as an output for PWM
	DDRD |= (1 << PD5) | (1 << PD4);

	TCCR1B |=  (1<< CS01); // Prescaler 8
	TCCR1A |=(1<<WGM10)|(1<<WGM11) | (1<<COM1A1) | (1<<COM1B1); //PRESCALER=1 MODE 14(FAST PWM)
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
	setup_PWM();
	
	// Enable global interrupts
	sei();

	// Main loop
	while (1) {
		
		if(receivedData == 0x01){
			DDRD |= (0 << PD6) | (1 << PD7);
			OCR1A = 255;
			OCR1B = 255;
		}
		else if(receivedData == 0x02){
			DDRD |= (1 << PD6) | (0 << PD7);
			OCR1A = 255;
			OCR1B = 255;
		}
		else if(receivedData == 0x03){
			DDRD |= (0 << PD6) | (0 << PD7);
			OCR1A = 255;
			OCR1B = 255;
		}
		else if(receivedData == 0x04){
			DDRD |= (1 << PD6) | (1 << PD7);
			OCR1A = 255;
			OCR1B = 255;
		}
		else if(receivedData == 0x00){
			OCR1A = 0;
			OCR1B = 0;
		}
		//dataToSend = 0x20;
		//SPI_send(dataToSend);
		
	}
	return 0;
}