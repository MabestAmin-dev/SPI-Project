#include <stdio.h>

// PD controller structure
typedef struct {
    double Kp;  // Proportional gain
    double Kd;  // Derivative gain
    double prevError;  // Previous error for derivative term
} PDController;

// Function to initialize the PD controller
void initPDController(PDController *pd, double Kp, double Kd) {
    pd->Kp = Kp;
    pd->Kd = Kd;
    pd->prevError = 0;
}

// Function to compute the control output
double computePD(PDController *pd, double sensorFL, double sensorFR, double sensorBR, double sensorBL) {
    // Calculate the error
    // Error for the front sensors
    double errorFront = sensorFL - sensorFR;

    double errorBack = sensorBL - sensorBR;

    // double error = setpoint - processVariable;

    // Proportional term
    double P = pd->Kp * errorFront;

    // Derivative term
    double D = pd->Kd * (errorFront - errorBack);

    // Calculate the control output
    double output = P + D;

    return output;
}

void getSensorData(){
    // code that will get us the sensor data 
    // sensorFL = ...
    // sensorFR = ...
    // sensorBL = ...
    // sensorBR = ...
}

int main() {

    // Från sensor lokaliserad FL (fron left) till sensor BL (back left) 2 på
    // 13.5 cm är längden mellan sido sensorerna till andra sidan, 20 cm
    // är hela robotens bredd

    // Example usage
    PDController pd;
    double setpoint = 50.0;
    double processVariable = 0.0;

    // Initialize PD controller with gains
    // Init with random constants that will be changed later
    initPDController(&pd, 10, 40);

    while(1){ 
        getSensorData();
        double computeBack = computePD(&pd, sensorFL, sensorFR, sensorBL, sensorBR);
    
    }

    // Simulate the control loop
    for (int i = 0; i < 100; i++) {
        // Simulate the process by updating the process variable
        // In a real system, this would be replaced with actual sensor readings
        processVariable += 0.5;

        // Compute PD control output
        double controlOutput = computePD(&pd, setpoint, processVariable);

        // Apply the control output to the system (in this example, just print it)
        printf("Time: %d, Control Output: %f\n", i, controlOutput);
    }

    return 0;
}
