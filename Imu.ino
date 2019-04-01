unsigned long ts=millis();
unsigned long tf=micros();

void MPU_init(){
/********** IMU initialization **************/
  Wire.begin();
  Wire.setClock(400000); 
  mpu.initialize();
  #ifdef DEBUG_OUTPUT
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  #endif
  devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    packetSize = mpu.dmpGetFIFOPacketSize();
  }else{
    #ifdef DEBUG_OUTPUT
      Serial.print(F("DMP Initialization failed (code "));
    #endif
  }
  mpu.setXAccelOffset(-1325);    
/*********************************************/
  get_MPU_data();//run this code for the first time to initialize the kalman filters.
}

void get_MPU_data(){

mpuIntStatus = mpu.getIntStatus();

// get current FIFO count
fifoCount = mpu.getFIFOCount();

// check for overflow (this should never happen unless our code is too inefficient)
if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
  // reset so we can continue cleanly
  mpu.resetFIFO();
  fifoCount = mpu.getFIFOCount();
  Serial.println(F("FIFO overflow!"));

  // otherwise, check for DMP data ready interrupt (this should happen frequently)
} else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
  // wait for correct available data length, should be a VERY short wait
  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

  // read a packet from FIFO
  mpu.getFIFOBytes(fifoBuffer, packetSize);
        
  // track FIFO count here in case there is > 1 packet available
  // (this lets us immediately read more without waiting for an interrupt)
  fifoCount -= packetSize;
  // display Euler angles in degrees

  if((micros()-tf)>1300){
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      mpu.dmpGetGyro(&angle_speed, fifoBuffer);  //Update only per 1300us, (~800Hz update rate)
    tf=micros();  
  }

  

            
  angleX = ypr[2] * 180/M_PI;
  angleY = ypr[1] * 180/M_PI;
  angleZ = ypr[0] * 180/M_PI;
  angleSpeedX = angle_speed.x;
  angleSpeedY = -angle_speed.y;
  angleSpeedZ = -angle_speed.z;

//  Serial.print(angleZ);
//  Serial.print("\t");
//  Serial.println(angleSpeedZ);
//  Serial.print("\n");


  /************************ kalman filter process ****************************/
  if(kalmanReady){
    double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    timer = micros();
    angleX = kalmanPitch.getAngle(angleX, angleSpeedX, dt);
    angleY = kalmanRoll.getAngle(angleY, angleSpeedY, dt);
    angleZ = kalmanYaw.getAngle(angleZ, angleSpeedZ, dt);
    angleSpeedX = kalmanPitch.getRate();
    angleSpeedY = kalmanRoll.getRate();
    angleSpeedZ = kalmanYaw.getRate();
  }else{
    kalmanPitch.setAngle(angleX);
    kalmanRoll.setAngle(angleY);
    kalmanYaw.setAngle(angleZ);
    kalmanReady = true;
  }
  /***************************************************************************/
//            Serial.print("gyro\t");

//            Serial.print(angleX);
//            Serial.print("\n");
//            Serial.print("\t");
//            Serial.print(gyroY);
//            Serial.print("\t");
//            Serial.println(gyroZ);
            
//            Serial.print("ypr\t");
//            Serial.print(angleX);
//            Serial.print("\t");
//            Serial.print(angleY);
//            Serial.print("\t");
//            Serial.println(angleZ);
            
    }
}
