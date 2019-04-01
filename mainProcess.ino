void setup() 
{  
  #ifdef DEBUG_OUTPUT
    Serial.begin(115200);
    while(!Serial);
    Serial.println("Debug Output ON");
  #endif
  MPU_init();
//  MPU.init();
  motors_initialize();
  rx_initialize();
  pid_initialize();
  #ifdef CALIBRATE
    motors_arm();
  #endif
  //wait for IMU YAW  to settle before beginning??? ~20s
}

void loop() 
{
  get_MPU_data();
//  updateSensorVal();
  control_update();
  #ifdef DEBUG_OUTPUT
    debug_process();
  #endif
  prev_time = micros();
}
