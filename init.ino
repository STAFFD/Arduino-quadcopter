
void setup() 
{  
  #ifdef DEBUG_OUTPUT
    Serial.begin(115200);
    while(!Serial);
    Serial.println("Debug Output ON");
  #endif

  MPU_init();
  motors_initialize();
  rx_initialize();
  pid_initialize();
  #ifdef CALIBRATE
    motors_arm();
  #endif
  Serial.println("start");
  //wait for IMU YAW  to settle before beginning??? ~20s
}
