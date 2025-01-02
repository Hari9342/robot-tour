#include <util/atomic.h>



const int enca[] = {3,2};
const int encb[] = {5,4};
const int pwm[] = {10,11};
const int in1[] = {12,13};
int enca_prev[] = {0,0};

long prevT = 0;
volatile int posi[] = {0,0};

long target[] = {0,0};

#define NMOTORS 2

class Controller{
  private:
    float kp, umax; // Parameters
  
  public:
  // Constructor
  Controller() : kp(1), umax(255){}

  // A function to set the parameters
  void setParams(float kpIn, float umaxIn){
    kp = kpIn; umax = umaxIn;
  }

  // A function to compute the control signal
  void evalu(int value, int target, int &pwr, int &dir){
    // error
    int e = target - value;
  
    // control signal
    float u = kp*e;
  
    // motor power
    pwr = (int) fabs(u);
    if( pwr > umax ){
      pwr = umax;
    }
  
    // motor direction
    dir = 1;
    if(u<0){
      dir = -1;
    }
  
  }
};

Controller Controller[NMOTORS];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);

  for(int k = 0; k < NMOTORS; k++){
    pinMode(enca[k],INPUT);
    pinMode(encb[k],INPUT);
    pinMode(pwm[k],OUTPUT);
    pinMode(in1[k],OUTPUT);
  }
  attachInterrupt(digitalPinToInterrupt(enca[0]),readEncoder<0>,CHANGE);
  attachInterrupt(digitalPinToInterrupt(enca[1]),readEncoder<1>,CHANGE);
}

void loop() {
  // put your main code here, to run repeatedly:
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;
    

    int pos[NMOTORS];
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
    for(int k = 0; k < NMOTORS; k++){
      pos[k] = posi[k];
    }
  }
  
  // loop through the motors
  for(int k = 0; k < NMOTORS; k++){
    int pwr, dir;
    // evaluate the control signal
    Controller[k].evalu(pos[k],target[k],pwr,dir);
    // signal the motor
    setMotor(dir,pwr,pwm[k],in1[k]);
  }
}



void setMotor(int dir, int pwmVal, int pwm, int in1){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
  }
  else{
    digitalWrite(in1,LOW);

  }
}



template <int j>
void readEncoder(){
  int enca_signal = digitalRead(enca[j]);
  int encb_signal = digitalRead(encb[j]);
  int enca_signal_prev = enca_prev[j];
  if((enca_signal_prev == LOW) && enca_signal==HIGH){
    if(encb_signal == LOW)
    {
      //Reverse
      posi[j]--;
    }
    else if(encb_signal == HIGH)
    {
      //Forward
      posi[j]++;
    }
  }
  enca_signal_prev = enca_signal;
}