
void imu_update() 
{
  Wire.requestFrom(ADDR_SLAVE_I2C, PACKET_SIZE);
  byte data[PACKET_SIZE];
  int i = 0;
  while(Wire.available())
  { 
    data[i] = Wire.read(); 
    i++;
  }
  
  // we use a c union to convert between byte[] and float
  union ROL_tag {byte b[4]; float fval;} ROL_Union; 
  union PIT_tag {byte b[4]; float fval;} PIT_Union; 
  union YAW_tag {byte b[4]; float fval;} YAW_Union;
  
  ROL_Union.b[0] = data[0];
  ROL_Union.b[1] = data[1];
  ROL_Union.b[2] = data[2];
  ROL_Union.b[3] = data[3];
  if(isnan(ROL_Union.fval) != 1)
  {
    angleX = ROL_Union.fval;
  }
  
  PIT_Union.b[0] = data[4];
  PIT_Union.b[1] = data[5];
  PIT_Union.b[2] = data[6];
  PIT_Union.b[3] = data[7];
  if(isnan(PIT_Union.fval) != 1)
  {
    angleY = PIT_Union.fval;
  }
  
  YAW_Union.b[0] = data[8];
  YAW_Union.b[1] = data[9];
  YAW_Union.b[2] = data[10];
  YAW_Union.b[3] = data[11];
  if(isnan(YAW_Union.fval) != 1)
  {
    angleZ = YAW_Union.fval;
  }
}




void MPU_init(){
  /******************************************************/
  //Serial.println("Initializing I2C devices...");
  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(20); // Wait for sensor to stabilize
  
  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];
  
  double pitch  = atan2(accY, accZ) * RAD_TO_DEG;
  kalmanPitch.setAngle(pitch);
  double roll  = atan2(accX, accZ) * RAD_TO_DEG;
  kalmanRoll.setAngle(roll);
  double yaw  = atan2(accX, accY) * RAD_TO_DEG;
  kalmanYaw.setAngle(yaw);
    
/*************************************************************************************/
}

void get_MPU_data(){
/************** retrieve MPU data ********************/
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];
/*******************************************************/

/************** kalman filter process ******************/
  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  double pitch  = atan2(accY, accZ) * RAD_TO_DEG;
  double roll  = atan2(accX, accZ) * RAD_TO_DEG;   // Calculate the angle based on accelarate meter
  double yaw  = atan2(accX, accY) * RAD_TO_DEG;

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = -gyroY / 131.0; // Convert to deg/s
  double gyroZrate = -gyroZ / 131.0; // Convert to deg/s

  angleX = kalmanPitch.getAngle(pitch, gyroXrate, dt);
  angleSpeedX = kalmanPitch.getRate();

  angleY = kalmanRoll.getAngle(roll, gyroYrate, dt);
  angleSpeedY = kalmanRoll.getRate();

  angleZ = kalmanYaw.getAngle(yaw, gyroZrate, dt);
  angleSpeedZ = kalmanYaw.getRate();
}
