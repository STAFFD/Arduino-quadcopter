void loop() 
{
  get_MPU_data();
  control_update();
  #ifdef DEBUG_OUTPUT
    debug_process();
  #endif
  prev_time = micros();
}
