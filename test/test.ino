
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include <PID_v1.h>
#include <Adafruit_MotorShield.h>


// Create the motor shield object with the default I2C address
Adafruit_MotorShield motorShield = Adafruit_MotorShield(); 

// Add left and right wheels as ports 1 and 2
Adafruit_DCMotor *left = motorShield.getMotor(2);
Adafruit_DCMotor *right = motorShield.getMotor(1);

// Global Variables to adjust tuning
double Throttle = .4;
double Lmultiplier = 1;
double Calibrationoffset = 0.;
double Sensitivity = 1;
int Mode = 0; // 2 is manual 0 is auto 1 is combined


//Global variables for PID input/output
double Setpoint, Input, Output = 0.0;

//Global PID tuning parameters
double Kp=3, Ki=0.1, Kd=0;

// Define pid class with pointers to global variables
PID pid(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Initialize measurement unit
MPU6050 mpu;


int16_t accZ, gyroY, gyroRate, last, last2, last3, last4, last5, trailAvg;
volatile float accAngle, gyroAngle=0, currentAngle, prevAngle=0, error, prevError=0, errorSum=0;
unsigned long currTime, prevTime=0, loopTime;
volatile int motorPower;

// Set active to true by default
bool Active = true;



// Trailing Average Filter
const int numReadings = 45;

float readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
float total = 0;                  // the running total
float average = 0;                // the average



void setup() {  
  
  // Initialize mpu and motor shield
  mpu.initialize();
  motorShield.begin();

  // Setup PID and redefine its output limits to center around 0
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-50.,50.);

  // Set sample rate of the PID loop to 100 times per second (Default 10)
  pid.SetSampleTime(10);

  // Begin serial at high baud rate to transfer commands quickly
  Serial.begin(115200);   

  // Sets all filter variables to 0
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }

}

void loop() {

  // Gathers Y axis acceleration
  float accY = map(mpu.getAccelerationZ(), -32768, 32767, -255.0, 255.0) + 29;

  // subtract the last reading:
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = accY;
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  average = total / numReadings;
  // send it to the computer as ASCII digits

  Serial.print(Output);
  Serial.print(',');
  Serial.println(average);
  delay(1);        // delay in between reads for stability


  // last = accY;
  // last2 = last;
  // last3 = last2;
  // last4 = last3;
  // last5 = last4;
  // accY = map(mpu.getAccelerationZ(), -32768, 32767, -255, 255) + 20;

  // trailAvg = (accY + last + last2 + last3 + last4 + last5)/6;
  // delay(300);

  // Serial.print(accY);
  // Serial.print(',');
  // Serial.println(trailAvg);


  // Set input to calibrated difference between sensors
  Input = average;

  pid.Compute();

  double value = motorSpeed(Output, Throttle);



}


/*
Takes an output from the PID controller and runs the left and right motors
@param output the output of the PID controller
@param throttle the net forward speed of the robot
@returns output the output of the PWM 
*/
double motorSpeed(double output, double throttle){

  double left;
  double right;
  // Remap throttle to pwm range (0-255)
  double newthrottle = throttle * 255.0;

  // Choose wheel direction based on sign of output
  if (output < 0) {
      // Map wheel velocity to the putput using the throttle and sensitivity
      right = -(Sensitivity * map(output, 25.,0. , 0.0, newthrottle));
      left = -(Sensitivity * map(output, 25.,0. , 0.0, newthrottle));
  }

  if (output >= 0) {
      // Map wheel velocity to the output using the throttle and sensitivity
      right = (Sensitivity * map(output, -25.,0. , 0.0, newthrottle));
      left = (Sensitivity * map(output, -25.,0. , 0.0, newthrottle));
  }
    
    // Run the motor control function
  runMotors(left,right);
    
  return output;
}
/*
Performs the motor speed and direaction assignments
@param leftSpeed, rightSpeed the desired motor speeds
*/
void runMotors( float leftSpeed, float rightSpeed){


  // Use to deactivate robot from CLI
  if (Active == true){

      // Keep motor values in bounds
      if (leftSpeed < -255 ) {leftSpeed = -255;}
      if (leftSpeed > 255 ) {leftSpeed = 255;}
      if (rightSpeed < -255 ) {rightSpeed = -255;}
      if (rightSpeed > 255 ) {rightSpeed = 255;}

      // Set motor speeds (can not be negative) and apply left multiplier
      left->setSpeed(abs(int(leftSpeed * Lmultiplier)));
      right->setSpeed(abs(int(rightSpeed)));
  
  }
  else{
      // Stop robot
      left->setSpeed(0);
      right->setSpeed(0);
  }

//    if( Printmotors){
//        // Print motor values to serial
//        Serial.print(leftSpeed);
//        Serial.print(", ");
//        Serial.println(rightSpeed);

//    }
//
//    if (Printsensors){
//        // Print sensor values to serial
//        Serial.print("A0: ");
//        Serial.println(analogRead(A0));
//        Serial.print("A1: ");
//        Serial.println(analogRead(A1));
//    }
    // Set wheel direction based on sign of input speed
    if (leftSpeed < 0){left->run(BACKWARD);}
    else{left->run(FORWARD);}
    if (rightSpeed < 0){right->run(BACKWARD);}
    else{right->run(FORWARD);}
}
