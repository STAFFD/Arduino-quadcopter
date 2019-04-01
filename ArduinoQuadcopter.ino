/*
  Arduino Quadcopter
  Author - STAFFD - Mar 24, 2019
*/
#include "Configuration.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Math.h>
#include <PID_v1.h>
#include <PinChangeInt.h>
#include <Servo.h> 
#include <Wire.h>
#include <Kalman.h>
#include <PIDCont.h>
#include <MPULib.h>

// Angles & angleSpeed
float angleX,angleY,angleZ;
float angleSpeedX, angleSpeedY, angleSpeedZ;

// RX Signals
int throttle=THROTTLE_RMIN;
volatile int rx_values[4]; // ROLL, PITCH, THROTTLE, YAW

// PID variables
double pid_roll_in,   pid_roll_out,   pid_roll_setpoint = 0;
double pid_pitch_in,  pid_pitch_out,  pid_pitch_setpoint = 0;
double pid_yaw_in,    pid_yaw_out,    pid_yaw_setpoint = 0;
  
// Motors -> white is front.
int m0, m1, m2, m3;

// Track loop time.
unsigned long prev_time = 0;

/****************** IMU Data ***********************/
double timer;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
VectorInt16 angle_speed;

float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

MPU6050 mpu;



//PIDCont PIDroll,PIDpitch,PIDyaw,PIDangleX,PIDangleY;
//MPULib MPU;
//
//unsigned long tp;
//float angles[2]={
//  0.0,0.0};
//float gx_aver=0;
//float gy_aver=0;
//float gz_aver=0;
/***************************************************/ 

/********** Create the kalman filter for IMU **********/
Kalman kalmanPitch;
Kalman kalmanRoll;
Kalman kalmanYaw;
bool kalmanReady = false;
/***************************************************/ 
