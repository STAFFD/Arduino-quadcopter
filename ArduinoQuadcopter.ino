/*
  Arduino Quadcopter
  Author - Sheldon Von - Mar 24, 2019
*/

#include "Configuration.h"
#include <Math.h>
#include <PID_v1.h>
#include <PinChangeInt.h>
#include <Servo.h> 
#include <Wire.h>
#include <Kalman.h>

// Angles & angleSpeed
float angleX,angleY,angleZ = 0.0;
float angleSpeedX, angleSpeedY, angleSpeedZ = 0.0;

// RX Signals
int throttle=THROTTLE_RMIN;
volatile int rx_values[4]; // ROLL, PITCH, THROTTLE, YAW

// PID variables
double pid_roll_in,   pid_roll_out,   pid_roll_setpoint = 0;
double pid_pitch_in,  pid_pitch_out,  pid_pitch_setpoint = 0;
double pid_yaw_in,    pid_yaw_out,    pid_yaw_setpoint = 0;
  
// Motors
int m0, m1, m2, m3;

// Track loop time.
unsigned long prev_time = 0;

/****************** IMU Data ***********************/
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
uint8_t i2cData[14]; // Buffer for I2C data
uint32_t timer;
/***************************************************/ 

/********** Create the kalman filter for IMU **********/
Kalman kalmanPitch;
Kalman kalmanRoll;
Kalman kalmanYaw;
/***************************************************/ 
