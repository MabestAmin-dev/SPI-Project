#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define SPI_DDR DDRB
#define SPI_PORT PORTB
#define DD_SS DDB4
#define DD_MOSI DDB5
#define DD_MISO DDB6
#define DD_SCK DDB7
#define MAX_IR 6
#define MAX_REFLEX 11

volatile uint8_t spiCounter = 0;
volatile uint8_t sensorSelect = 0x00;


// Global variable to store data to be sent
volatile uint8_t dataToSend = 0;

// Global variable to store received data
volatile uint8_t receivedData = 0;

volatile uint16_t irArray[MAX_IR];
volatile uint16_t twoDIR [MAX_IR][10];
volatile uint8_t irCounter = 0;
volatile uint16_t reflexArray[MAX_REFLEX];


volatile uint16_t gyroValue = 0;

volatile uint16_t encoderValue = 0;

volatile int lastEncoderValue = 0;

volatile uint16_t reflexCoverage = 0;

volatile uint16_t reflexCoverage_toSend = 0;

volatile uint16_t reflexDevider = 500;

volatile uint16_t reflex_high = 0;
volatile uint16_t reflex_low = 0;

volatile uint16_t button = 1;

volatile uint16_t button_toSend = 1;

volatile uint8_t left_buttonCounter = 2;

volatile uint8_t right_buttonCounter = 2;

volatile uint8_t new_right_press = 0;

volatile uint8_t new_left_press = 0;

// Comparison function for qsort
int compare_uint16_t(const void *a, const void *b) {
	return (*(uint16_t *)a - *(uint16_t *)b);
}


void SPI_init_slave() {
	// Set SS, MISO, and SCK as input
	SPI_DDR &= ~((1 << DD_SS) | (1 << DD_MISO) | (1 << DD_SCK));
	SPI_DDR |= (1 << DD_MISO);

	// Enable SPI, SPI interrupt, set as a slave, and set SCLK freq to f/16
	SPCR = (1 << SPE) | (1 << SPIE);
}


/*
 * initLidar: Initializes the Lidar sensor.
 * - Sets the reference voltage for the ADC (Analog-to-Digital Converter).
 * - Enables ADC and sets the ADC prescaler for appropriate conversion speed.
 * - Configures PC7 as an output pin and sets it high.
 * - Disables the digital input buffer for ADC0.
 */
 
void initIR() {

    ADMUX = (1 << REFS0 );
    
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);

    DDRC |= (1 << PC7);

    PORTC |= (1 << PC7);

    DIDR0 = (1 << ADC0D);
}


/*
 * initReflex: Initializes the Reflex sensor.
 * - Sets the reference voltage for the ADC.
 * - Enables ADC and sets the ADC prescaler for appropriate conversion speed.
 * - Disables the digital input buffer for ADC0.
 * - Configures PC2-PC6 as output pins, sets PC6 high, and sets PC6 high again.
 */

void initReflex() {
    ADMUX = (1 << REFS0);
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);
    DIDR0 = (1 << ADC0D);

    DDRC |= (1 << PC6);  // Enable pin
    DDRD |= (1 << PD0) | (1 << PD5);  // A0 and A1
    DDRB |= (1 << PB0) | (1 << PB1);  // A3 and A2
    DDRA &= ~(1 << PA7);  // Analog In

    // Set Enable pin high
    PORTC |= (1 << PC6);
}

void initButtons() {
    DDRC &= ~((1 << PC0) | (1 << PC1));
}

/*
 * initGyro: Initializes the Gyro sensor.
 * - Sets the reference voltage for the ADC.
 * - Enables ADC and sets the ADC prescaler for appropriate conversion speed.
 * - Disables the digital input buffer for ADC0.
 */

void initGyro() {
   
    //ADMUX = (1 << REFS0 );
//
    //ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);

    DIDR0 = (1 << ADC0D);
}


void initEncoder() {
	
	 DDRD &= ~(1 << PD6);

}

/*
 * readSensor: Reads the analog value from the specified ADC channel.
 * - Updates the ADC multiplexer to select the desired channel.
 * - Starts the ADC conversion and waits for it to complete.
 * - Returns the 16-bit ADC result.
 */

uint16_t readADC(uint8_t channel) {

    ADMUX = (ADMUX & 0xF8) | (channel & 0x07);

    ADCSRA |= (1 << ADSC);

    while (ADCSRA & (1 << ADSC));

    return ADC;
}

int readInt(uint8_t channel){
	
	if (PIND & (1 << PD6)) {
		return 1;
	}
	else {
		return 0;
	}
}

void readLeftButton() {
    if (PINC & (1 << PC0) && !(new_left_press)) { // Check if the button is pressed
        // Toggle the 8th bit of button
		uint16_t reflex_avg = 0;
		for (int i = 0; i < MAX_REFLEX; i++) {
			reflex_avg += reflexArray[i];
		}
		
		reflex_high = reflex_avg / MAX_REFLEX;
		
		reflexDevider = (reflex_high + reflex_low) / 2;
		
        if (left_buttonCounter%2 == 0) {
            // If the 8th bit is not set, set it
            button |= 0x0100;
			left_buttonCounter++;
        } else {
            // If the 8th bit is set, clear it
            button &= 0x00FF;
			left_buttonCounter++;
        }
		new_left_press = 1;
    } else if (!(PINC & (1 << PC0)) && new_left_press) {
		new_left_press = 0;
	}
}

void readRightButton() {
    if (PINC & (1 << PC1) && !(new_right_press)) { // Check if the button is pressed	
		    
		uint16_t reflex_avg = 0;
		for (int i = 0; i < MAX_REFLEX; i++) {
			reflex_avg += reflexArray[i];
		}
		
		reflex_low = reflex_avg / MAX_REFLEX;
			
		reflexDevider = (reflex_high + reflex_low) / 2;	
		
	    if (right_buttonCounter%2 == 0) {
		    button |= 0x0001;
			right_buttonCounter++;
		    } else {
				
		    button &= 0xFF00;
			right_buttonCounter++;

	    }
		new_right_press = 1;
    } else if (!(PINC & (1 << PC1)) && new_right_press) {
		new_right_press = 0;
    }
}

uint8_t SPI_receive() {
	return SPDR;
}

void SPI_send(uint8_t data) {
	SPDR = data;
}


ISR(SPI_STC_vect) {
	uint8_t received = SPI_receive();
	
    if (spiCounter == 0) {
        sensorSelect = received;
        if(sensorSelect == 0x06){
            SPI_send(gyroValue & 0xFF);
        } 
		else if (sensorSelect == 0x07) {
			SPI_send(reflexCoverage_toSend & 0xFF);
		}
		else if (sensorSelect == 0x08) {
			SPI_send(encoderValue & 0xFF);
        } 
		else if (sensorSelect == 0x09) {
            SPI_send(button_toSend & 0xFF);
		}
        else {
            SPI_send((irArray[sensorSelect] & 0xFF));
        }
        spiCounter++;
    } 
	else if (spiCounter == 1) {
		if(sensorSelect == 0x06) {
            SPI_send(((gyroValue >> 8) & 0xFF));
        }
		else if (sensorSelect == 0x07) {
			SPI_send(((reflexCoverage_toSend >> 8) & 0xFF));
		}
		else if (sensorSelect == 0x08) {
			SPI_send(((encoderValue >> 8) & 0xFF));
			encoderValue = 0;
		}
        else if (sensorSelect == 0x09) {
            SPI_send((button_toSend >> 8) & 0xFF);
		}
        else {
            SPI_send((irArray[sensorSelect] >> 8) & 0xFF);
        }
		spiCounter++;
    } 
	else if (spiCounter == 2) {
        SPI_send(0x00);
        spiCounter = 0;
    }
}


int main() {
    // Initialize the the sensors
    initGyro();
    initIR();
    initReflex();
    initEncoder();
	SPI_init_slave();
	
	sei();

    
    
	uint16_t irSensorsTestBoard[6] = {0, 0, 0, 0, 0, 0};
	
	uint16_t irSensors[6] = {4, 1, 2, 3, 5, 6};
    //uint16_t irSensors[6] = {0, 1, 2, 3, 4, 6};

    while (1) {

		
		int test = readInt(1);
		
		if (lastEncoderValue == 1 && readInt(1) == 1){
			encoderValue = encoderValue + 1;
			lastEncoderValue = 0;
		}
		else if (lastEncoderValue ==  0 && readInt(1) == 0){
			lastEncoderValue = 1;
		}
		
        for (int i = 0; i < MAX_IR; i++) {

            irArray[i] = readADC(irSensors[i]);
			
            // This test array is used for the second test board
            // Change to the first to test on the main board
			// irArray[i] = readADC(irSensorsTestBoard[i]);
            
			irArray[i] = (2914 / (irArray[i] + 5)) - 1;
        }
		
		if(irCounter == 9){
			irCounter = 0;
		}
		else{
			irCounter++;
		}

		reflexCoverage = 0;
        for (uint16_t i = 1; i <= MAX_REFLEX; i++) {
            PORTD = (((i & 0x01) << PD0) | (((i >> 1) & 0x01) << PD5));
            PORTB = ((((i >> 2) & 0x01) << PB1) | (((i >> 3) & 0x01) << PB0));
            PORTC |= (1 << PC6);
            reflexArray[i - 1] = readADC(7);
            PORTC &= ~(1 << PC6);

            if (reflexArray[i - 1] > reflexDevider) {
                reflexCoverage++;
            }
        }

        reflexCoverage_toSend = reflexCoverage;
		
        readLeftButton();
        readRightButton();

        button_toSend = button;
		
        gyroValue = readADC(0);
    }    
}