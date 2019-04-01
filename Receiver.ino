
volatile int pwm_start_time[4];
static int lastInterruptPin = PIN_RX_THROTTLE;
uint8_t interrupted_pin;

void rx_initialize() {

  pinMode(PIN_RX_THROTTLE, INPUT); digitalWrite(PIN_RX_THROTTLE, HIGH);
  PCintPort::attachInterrupt(PIN_RX_THROTTLE, &rising, RISING);
  
  pinMode(PIN_RX_ROLL, INPUT); digitalWrite(PIN_RX_ROLL, HIGH);
  PCintPort::attachInterrupt(PIN_RX_ROLL, &rising, RISING);

  pinMode(PIN_RX_PITCH, INPUT); digitalWrite(PIN_RX_PITCH, HIGH);
  PCintPort::attachInterrupt(PIN_RX_PITCH, &rising, RISING);
    
  pinMode(PIN_RX_YAW, INPUT); digitalWrite(PIN_RX_YAW, HIGH);
  PCintPort::attachInterrupt(PIN_RX_YAW, &rising, RISING); 
}

void rising(){
  interrupted_pin=PCintPort::arduinoPin;
  PCintPort::attachInterrupt(interrupted_pin, &falling, FALLING);
  switch(interrupted_pin) {
      case PIN_RX_ROLL :{
        pwm_start_time[0] = micros();
        lastInterruptPin = PIN_RX_ROLL;
      }
      case PIN_RX_PITCH : {
        pwm_start_time[1] = micros();
        lastInterruptPin = PIN_RX_PITCH;
      }
      case PIN_RX_THROTTLE : {
        pwm_start_time[2] = micros();
        lastInterruptPin = PIN_RX_THROTTLE;
      }
      case PIN_RX_YAW : {
        pwm_start_time[3] = micros();
        lastInterruptPin = PIN_RX_YAW;
      }
  }
}

void falling() {
  interrupted_pin=PCintPort::arduinoPin;
  if(interrupted_pin == lastInterruptPin){
      PCintPort::attachInterrupt(interrupted_pin, &rising, RISING);
      switch(interrupted_pin) {
        case PIN_RX_ROLL : rx_values[0] = micros() - pwm_start_time[0];
        case PIN_RX_PITCH : rx_values[1] = micros() - pwm_start_time[1];
        case PIN_RX_THROTTLE : rx_values[2] = micros() - pwm_start_time[2];
        case PIN_RX_YAW : rx_values[3] = micros() - pwm_start_time[3];
    }
  }
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
