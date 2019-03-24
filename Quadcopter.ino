/*
  Arduino Quadcopter
  Author - Ben Ripley - Aug 8, 2014
*/

#include "Configuration.h"
#include <Math.h>
#include <PID_v1.h>
#include <PinChangeInt.h>
#include <Servo.h> 
#include <Wire.h>

// Angles
float angleX,angleY,angleZ = 0.0;

// RX Signals
int throttle=THROTTLE_RMIN;
volatile int rx_values[4]; // ROLL, PITCH, THROTTLE, YAW

// PID variables
double pid_roll_in,   pid_roll_out,   pid_roll_setpoint = 0;
double pid_pitch_in,  pid_pitch_out,  pid_pitch_setpoint = 0;
double pid_yaw_in,    pid_yaw_out,    pid_yaw_setpoint = 0;
  
// Motors
int m0, m1, m2, m3; // Front, Right, Back, Left

// Track loop time.
unsigned long prev_time = 0;

Servo motor0;
Servo motor1;
Servo motor2;
Servo motor3;

void motors_initialize(){
  pinMode(9, OUTPUT);
  pinMode(PIN_MOTOR1, OUTPUT);
  pinMode(PIN_MOTOR2, OUTPUT);
  pinMode(PIN_MOTOR3, OUTPUT);
  motor0.attach(9);
  motor1.attach(PIN_MOTOR1);
  motor2.attach(PIN_MOTOR2);
  motor3.attach(PIN_MOTOR3);
  motor0.writeMicroseconds(1500);
  motor1.writeMicroseconds(MOTOR_ZERO_LEVEL);
  motor2.writeMicroseconds(MOTOR_ZERO_LEVEL);
  motor3.writeMicroseconds(MOTOR_ZERO_LEVEL);
  while(true);
}


void setup() 
{  
  #ifdef DEBUG_OUTPUT
    Serial.begin(115200);
    while(!Serial);
    Serial.println("Debug Output ON");
  #endif
  
  motors_initialize();
//  leds_initialize();
  rx_initialize();
  pid_initialize();
  motors_arm();
  Serial.println("start");
  //wait for IMU YAW  to settle before beginning??? ~20s
}

void loop() 
{
  imu_update();
  control_update();

  #ifdef DEBUG_OUTPUT
    debug_process();
  #endif
  prev_time = micros();
}
