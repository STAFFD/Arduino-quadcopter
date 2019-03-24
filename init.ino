
void setup() 
{  
  #ifdef DEBUG_OUTPUT
    Serial.begin(115200);
    while(!Serial);
    Serial.println("Debug Output ON");
  #endif
  
  motors_initialize();
//  leds_initialize();
  MPU_init();
  rx_initialize();
  pid_initialize();
  motors_arm();
  Serial.println("start");
  //wait for IMU YAW  to settle before beginning??? ~20s
}
