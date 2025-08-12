#include <avr/io.h>
#include <avr/interrupt.h>
// #include <util/delay.h>

#define SPI_DDR DDRB
#define SPI_PORT PORTB
#define DD_SS DDB4
#define DD_MOSI DDB5
#define DD_MISO DDB6
#define DD_SCK DDB7
#define MAX_SPEED 300
#define MIN_SPEED 10



volatile double Kp = 0;
volatile double Kd = 0;
volatile double prevError = 0;


volatile int receive_counter = 0;
volatile int pd_counter = 0;
volatile uint8_t pd_recieved = 0x00;


volatile uint8_t direction = 0x00;

volatile uint16_t front_left_sensor = 0;
volatile uint16_t front_right_sensor = 0;
volatile uint16_t back_left_sensor = 0;
volatile uint16_t back_right_sensor = 0;

volatile uint16_t leftMotorSpeed = 250;
volatile uint16_t rightMotorSpeed = 250;

// Global variable to store data to be sent
volatile uint8_t dataToSend = 0;

// Global variable to store received data
volatile uint8_t receivedData = 0;

volatile int counter = 1;

volatile double prevOutput = 0;

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
	DDRC |= (1 << PC7) | (1 << PC1);

	TCCR1B |=  (1<< CS01); // Prescaler 8
	TCCR1A |=(1<<WGM10)|(1<<WGM11) | (1<<COM1A1) | (1<<COM1B1); //PRESCALER=1 MODE 14(FAST PWM)
	
	//OCR1A = 39999;
	//OCR1B = 39999;
	
}

void setup_PWM_gripen() {
	// Set PD7 (OC2A) pin as an output for PWM
	DDRD |= (1 << PD7);

	// Configure Timer 2 for Fast PWM with OC2A output
	TCCR2A |= (1<<WGM20)|(1<<WGM21) | (1<<COM2A1) | (1<<COM2B1);
	TCCR2B |= (1 << CS20)|(1 << CS22)|(1 << CS21); // Prescaler 1024 = 33 ms period time
	
	//OCR2A = 20;
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




// Function to compute the control output
double computePD(uint16_t sensorFL, uint16_t sensorFR, uint16_t sensorBL, uint16_t sensorBR) {
	// Calculate the error
	// Error for the front sensors
	double double_sensorFL = (double)sensorFL;
	double double_sensorFR = (double)sensorFR;
	double double_sensorBL = (double)sensorBL;
	double double_sensorBR = (double)sensorBR;
	
	double leftMidError = (double_sensorFL + double_sensorBL)/2;
	double rightMidError = (double_sensorFR + double_sensorBR)/2;
	
	//double errorFront = sensorFL - sensorFR;
	//
	//double errorBack = sensorBL - sensorBR;
	
	double error = leftMidError - rightMidError;
	// Proportional term
	double P = Kp * error;
	double D = 0;

	//// Derivative term
	//double D = pd->Kd * (error - pd->prevError);
	//pd->prevError = error;
	if(error > 0){
		D = Kd * (sensorFR - sensorBR);
	}
	else{
		D = Kd * (sensorFL - sensorBL);
	}
	


	// Calculate the control output
	double output = P + D;
	
	return output;
}


void adjustMotors(double controlOutput) {
	if (controlOutput != 0){
		// Example motor adjustment logic
		// Assume a simple logic: if control output is positive, turn right; if negative, turn left

		leftMotorSpeed -= controlOutput;
		rightMotorSpeed += controlOutput;
		
		//if (prevOutput < 0 && controlOutput > 0) {
		//leftMotorSpeed = 300;
		//rightMotorSpeed = 325;
		//}
		//else if (prevOutput > 0 && controlOutput < 0) {
		//leftMotorSpeed = 325;
		//rightMotorSpeed = 300;
		//}
		
		prevOutput = controlOutput;

		// Apply the motor speed limits
		leftMotorSpeed = (leftMotorSpeed < MIN_SPEED) ? MIN_SPEED : (leftMotorSpeed > MAX_SPEED) ? MAX_SPEED : leftMotorSpeed;
		rightMotorSpeed = (rightMotorSpeed < MIN_SPEED) ? MIN_SPEED : (rightMotorSpeed > MAX_SPEED) ? MAX_SPEED : rightMotorSpeed;
	}
}

ISR(SPI_STC_vect) {
	uint8_t received = SPI_receive();
	processReceivedData(received);
	if((receive_counter == 0 && received == 0x07) || pd_recieved == 0x07){
		pd_recieved = received;
		if(pd_counter == 1){
			Kp = received;
			pd_counter = 0;
			pd_recieved = 0x00;
			leftMotorSpeed = 250;
			rightMotorSpeed = 250;
		}
		else{
			pd_counter++;
		}
	}
	else if((receive_counter == 0 && received == 0x08) || pd_recieved == 0x08){
		pd_recieved = received;
		if(pd_counter == 1){
			Kd = received;
			pd_counter = 0;
			pd_recieved = 0x00;
			leftMotorSpeed = 250;
			rightMotorSpeed = 250;
		}
		else{
			pd_counter++;
		}
	}
	else {
		if (receive_counter == 0) {
			direction = receivedData;
			receive_counter++;
		}
		else if (receive_counter == 1) {
			front_left_sensor = 0;
			front_left_sensor = ((uint16_t)receivedData << 8) & 0xFF;
			receive_counter++;
		}
		else if (receive_counter == 2) {
			front_left_sensor = front_left_sensor | receivedData;
			receive_counter++;
		}
		else if (receive_counter == 3) {
			front_right_sensor = 0;
			front_right_sensor = ((uint16_t)receivedData << 8) & 0xFF;
			receive_counter++;
		}
		else if (receive_counter == 4) {
			front_right_sensor = front_right_sensor | receivedData;
			receive_counter++;
		}
		else if (receive_counter == 5) {
			back_left_sensor = 0;
			back_left_sensor = ((uint16_t)receivedData << 8) & 0xFF;
			receive_counter++;
		}
		else if (receive_counter == 6) {
			back_left_sensor = back_left_sensor | receivedData;
			receive_counter++;
		}
		else if (receive_counter == 7) {
			back_right_sensor = 0;
			back_right_sensor = ((uint16_t)receivedData << 8) & 0xFF;
			receive_counter++;
		}
		else if (receive_counter == 8) {
			back_right_sensor = back_right_sensor | receivedData;
			receive_counter = 0;
		}
	}
	
}


void setGripen(uint16_t value){
	OCR2A = value; // 50% duty cycle initially
}


int main() {
	// Initialize SPI slave
	SPI_init_slave();
	setup_PWM();
	setup_PWM_gripen(); // Initialize PWM

	// MotorSpeed motorSpeed;
	
	Kp = 10;
	Kd = 10;
	
	double computeFront;
	
	// Enable global interrupts
	sei();
	

	// Main loop
	while (1) {
		computeFront = computePD(front_left_sensor, front_right_sensor, back_left_sensor, back_right_sensor);
		adjustMotors(computeFront);
		
		if(direction == 0x01){
			PORTC = (1 << PC7) | (0 << PC1);
			OCR1A = 380;
			OCR1B = 380;
			leftMotorSpeed = 250;
			rightMotorSpeed = 250;
		}
		else if(direction == 0x02){
			PORTC = (0 << PC7) | (1 << PC1);
			OCR1A = 380;
			OCR1B = 380;
			leftMotorSpeed = 250;
			rightMotorSpeed = 250;			
		}
		else if(direction == 0x03){
			PORTC = (1 << PC7) | (1 << PC1);
			OCR1A = leftMotorSpeed; // motorSpeed.leftMotorSpeed;
			OCR1B = rightMotorSpeed;
		}
		else if(direction == 0x04){
			PORTC = (0 << PC7) | (0 << PC1);
			OCR1A = leftMotorSpeed; // motorSpeed.leftMotorSpeed;
			OCR1B = rightMotorSpeed;
		}
		else if(direction == 0x00){
			OCR1A = 0;
			OCR1B = 0;
			leftMotorSpeed = 250;
			rightMotorSpeed = 250;
		}
		else if (direction == 0x05) {
			setGripen(9);
		}
		else if (direction == 0x06) {
			setGripen(12);
		}
		
	}
	return 0;
}
