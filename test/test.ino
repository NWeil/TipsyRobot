
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
double Throttle = .15;
double Lmultiplier = 1;
double Calibrationoffset = 0.;
double Sensitivity = 1;
int Mode = 0; // 2 is manual 0 is auto 1 is combined


//Global variables for PID input/output
double Setpoint, Input, Output = 0.0;

//Global PID tuning parameters
double Kp=3, Ki=5, Kd=0.01;

// Define pid class with pointers to global variables
PID pid(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Initialize measurement unit
MPU6050 mpu;


int16_t accY, accZ, gyroY, gyroRate;
volatile float accAngle, gyroAngle=0, currentAngle, prevAngle=0, error, prevError=0, errorSum=0;
unsigned long currTime, prevTime=0, loopTime;
volatile int motorPower;

// Set active to true by default
bool Active = true;



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

}

void loop() {

  // Set variables for timing
  currTime = millis();
  loopTime = currTime - prevTime;
  prevTime = currTime;
  
  // Get gyro and acceleration data
  gyroY = mpu.getRotationY();
  accY = mpu.getAccelerationY();
  accZ = mpu.getAccelerationZ(); 

  // map gyro data to +- 250 and calculate angle based on milisecond loop time
  gyroRate = map(gyroY, -32768, 32767, -250, 250);
  gyroAngle = gyroAngle + (float)gyroRate*loopTime/1000;

  // Calculate angle based on accelerometer data
  // TODO: make accAngle centered around 0

  accAngle = atan2(accY, accZ)*RAD_TO_DEG;

  // combine gyro and accelerometer readings
  currentAngle = 0.9934*(prevAngle + gyroAngle) + 0.0066*(accAngle);

  accY = map(mpu.getAccelerationZ(), -32768, 32767, -255, 255) + 20;


  Serial.println(accY);
  // Serial.print(',');
  // Serial.println(accZ);

//  Serial.println(currentAngle);


  // Set input to calibrated difference between sensors
  Input = accY;

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
