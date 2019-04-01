
volatile int pwm_start_time[4];
uint8_t latest_interrupted_pin;

void rx_initialize() {
//  pinMode(PIN_RX_ROLL, INPUT); digitalWrite(PIN_RX_ROLL, HIGH);
//  PCintPort::attachInterrupt(PIN_RX_ROLL, &rising, RISING);
//
//  pinMode(PIN_RX_PITCH, INPUT); digitalWrite(PIN_RX_PITCH, HIGH);
//  PCintPort::attachInterrupt(PIN_RX_PITCH, &rising, RISING);
  
  pinMode(PIN_RX_THROTTLE, INPUT); 
  digitalWrite(PIN_RX_THROTTLE, HIGH);
  PCintPort::attachInterrupt(PIN_RX_THROTTLE, &throttle_Interrupt, CHANGE);
  
//  pinMode(PIN_RX_YAW, INPUT); digitalWrite(PIN_RX_YAW, HIGH);
//  PCintPort::attachInterrupt(PIN_RX_YAW, &rising, RISING); 
}

void throttle_Interrupt(){
   if(!interruptLock) {
    if(PIN_RX_THROTTLE & B00000010) pwm_start_time[2] = micros();
    else rx_values[2] = micros() - pwm_start_time[2];
   }
}
void acquireLock(){
  interruptLock = true; 
}

void releaseLock(){
  interruptLock = false;
}

void pitch_Interrupt(){
   if(!interruptLock) rx_values[1] = micros() - pwm_start_time[1];
   pwm_start_time[1] = micros(); 
}

void roll_Interrupt(){
   if(!interruptLock) rx_values[0] = micros() - pwm_start_time[0];
   pwm_start_time[0] = micros(); 
}

void yaw_Interrupt(){
   if(!interruptLock) rx_values[3] = micros() - pwm_start_time[3];
   pwm_start_time[3] = micros(); 
}

void rising(){
  latest_interrupted_pin=PCintPort::arduinoPin;
  PCintPort::attachInterrupt(latest_interrupted_pin, &falling, FALLING);
  switch(latest_interrupted_pin) {
      case PIN_RX_ROLL : pwm_start_time[0] = micros();
      case PIN_RX_PITCH : pwm_start_time[1] = micros();
      case PIN_RX_THROTTLE : pwm_start_time[2] = micros();
      case PIN_RX_YAW : pwm_start_time[3] = micros();
  }
}

void falling() {
  latest_interrupted_pin=PCintPort::arduinoPin;
  PCintPort::attachInterrupt(latest_interrupted_pin, &rising, RISING);
    switch(latest_interrupted_pin) {
      case PIN_RX_ROLL : rx_values[0] = micros() - pwm_start_time[0];
      case PIN_RX_PITCH : rx_values[1] = micros() - pwm_start_time[1];
      case PIN_RX_THROTTLE : rx_values[2] = micros() - pwm_start_time[2];
      case PIN_RX_YAW : rx_values[3] = micros() - pwm_start_time[3];
  }
}
