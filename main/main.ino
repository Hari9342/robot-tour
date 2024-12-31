#include <util/atomic.h>






// A class to compute the control signal
class SimplePID{
  private:
    float kp, kd, ki, umax; // Parameters
    float eprev, eintegral; // Storage

  public:
  // Constructor
  SimplePID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0){}

  // A function to set the parameters
  void setParams(float kpIn, float kdIn, float kiIn, float umaxIn){
    kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
  }

  // A function to compute the control signal
  void evalu(int value, int target, float deltaT, int &pwr, int &dir){
    // error
    int e = target - value;
  
    // derivative
    float dedt = (e-eprev)/(deltaT);
  
    // integral
    eintegral = eintegral + e*deltaT;
  
    // control signal
    float u = kp*e + kd*dedt + ki*eintegral;
  
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
  
    // store previous error
    eprev = e;
  }
  
};

// How many motors
#define NMOTORS 2

// Pins
const int enca[] = {3,2};
const int encb[] = {5,4};
const int pwm[] = {10,11};
const int in1[] = {12,13};


// Globals
long prevT = 0;
volatile int posi[] = {0,0};

// PID class instances
SimplePID pid[NMOTORS];


float target_f[] = {0.0,0.0};
long target[] = {0,0};

int direction = 0;

void setTarget(float t, float deltat){
  
  float positionChange[2] = {0, 0};
  float pulsesPerTurn = 960;
  float pulsesPerMeter = pulsesPerTurn*3.9788;

  t = fmod(t,124); // time is in seconds
  float velocity = 0.16; // m/s
  if(t < 3){
    
  }else if(t<4.9625){
positionChange[0] = (velocity)*deltat*pulsesPerMeter;
positionChange[1] = (velocity)*deltat*pulsesPerMeter;
}
else if(t<5.286475){
positionChange[0] = -(velocity)*deltat*pulsesPerMeter;
positionChange[1] = (velocity)*deltat*pulsesPerMeter;
}
else if(t<8.411475){
positionChange[0] = (velocity)*deltat*pulsesPerMeter;
positionChange[1] = (velocity)*deltat*pulsesPerMeter;
}
else if(t<8.73545){
positionChange[0] = (velocity)*deltat*pulsesPerMeter;
positionChange[1] = -(velocity)*deltat*pulsesPerMeter;
}
else if(t<11.86045){
positionChange[0] = (velocity)*deltat*pulsesPerMeter;
positionChange[1] = (velocity)*deltat*pulsesPerMeter;
}
else if(t<12.184425000000001){
positionChange[0] = (velocity)*deltat*pulsesPerMeter;
positionChange[1] = -(velocity)*deltat*pulsesPerMeter;
}
else if(t<18.434425){
positionChange[0] = (velocity)*deltat*pulsesPerMeter;
positionChange[1] = (velocity)*deltat*pulsesPerMeter;
}
else if(t<18.7584){
positionChange[0] = (velocity)*deltat*pulsesPerMeter;
positionChange[1] = -(velocity)*deltat*pulsesPerMeter;
}
else if(t<21.8834){
positionChange[0] = (velocity)*deltat*pulsesPerMeter;
positionChange[1] = (velocity)*deltat*pulsesPerMeter;
}
else if(t<22.207375000000003){
positionChange[0] = -(velocity)*deltat*pulsesPerMeter;
positionChange[1] = (velocity)*deltat*pulsesPerMeter;
}
else if(t<23.394875000000003){
positionChange[0] = (velocity)*deltat*pulsesPerMeter;
positionChange[1] = (velocity)*deltat*pulsesPerMeter;
}
else if(t<24.482375){
positionChange[0] = -(velocity)*deltat*pulsesPerMeter;
positionChange[1] = -(velocity)*deltat*pulsesPerMeter;
}
else if(t<24.806350000000002){
positionChange[0] = -(velocity)*deltat*pulsesPerMeter;
positionChange[1] = (velocity)*deltat*pulsesPerMeter;
}
else if(t<34.18135){
positionChange[0] = (velocity)*deltat*pulsesPerMeter;
positionChange[1] = (velocity)*deltat*pulsesPerMeter;
}
else if(t<34.505325){
positionChange[0] = -(velocity)*deltat*pulsesPerMeter;
positionChange[1] = (velocity)*deltat*pulsesPerMeter;
}
else if(t<40.755325){
positionChange[0] = (velocity)*deltat*pulsesPerMeter;
positionChange[1] = (velocity)*deltat*pulsesPerMeter;
}
else if(t<41.079299999999996){
positionChange[0] = -(velocity)*deltat*pulsesPerMeter;
positionChange[1] = (velocity)*deltat*pulsesPerMeter;
}
else if(t<44.204299999999996){
positionChange[0] = (velocity)*deltat*pulsesPerMeter;
positionChange[1] = (velocity)*deltat*pulsesPerMeter;
}
else if(t<44.528274999999994){
positionChange[0] = (velocity)*deltat*pulsesPerMeter;
positionChange[1] = -(velocity)*deltat*pulsesPerMeter;
}
else if(t<47.653274999999994){
positionChange[0] = (velocity)*deltat*pulsesPerMeter;
positionChange[1] = (velocity)*deltat*pulsesPerMeter;
}
else if(t<47.97724999999999){
positionChange[0] = (velocity)*deltat*pulsesPerMeter;
positionChange[1] = -(velocity)*deltat*pulsesPerMeter;
}
else if(t<49.16474999999999){
positionChange[0] = (velocity)*deltat*pulsesPerMeter;
positionChange[1] = (velocity)*deltat*pulsesPerMeter;
}
else if(t<50.25224999999999){
positionChange[0] = -(velocity)*deltat*pulsesPerMeter;
positionChange[1] = -(velocity)*deltat*pulsesPerMeter;
}
else if(t<50.57622499999999){
positionChange[0] = (velocity)*deltat*pulsesPerMeter;
positionChange[1] = -(velocity)*deltat*pulsesPerMeter;
}
else if(t<53.70122499999999){
positionChange[0] = (velocity)*deltat*pulsesPerMeter;
positionChange[1] = (velocity)*deltat*pulsesPerMeter;
}
else if(t<54.025199999999984){
positionChange[0] = (velocity)*deltat*pulsesPerMeter;
positionChange[1] = -(velocity)*deltat*pulsesPerMeter;
}
else if(t<60.275199999999984){
positionChange[0] = (velocity)*deltat*pulsesPerMeter;
positionChange[1] = (velocity)*deltat*pulsesPerMeter;
}
else if(t<60.59917499999998){
positionChange[0] = (velocity)*deltat*pulsesPerMeter;
positionChange[1] = -(velocity)*deltat*pulsesPerMeter;
}
else if(t<63.72417499999998){
positionChange[0] = (velocity)*deltat*pulsesPerMeter;
positionChange[1] = (velocity)*deltat*pulsesPerMeter;
}
else if(t<64.04814999999998){
positionChange[0] = (velocity)*deltat*pulsesPerMeter;
positionChange[1] = -(velocity)*deltat*pulsesPerMeter;
}
else if(t<65.23564999999998){
positionChange[0] = (velocity)*deltat*pulsesPerMeter;
positionChange[1] = (velocity)*deltat*pulsesPerMeter;
}
else if(t<66.32314999999998){
positionChange[0] = -(velocity)*deltat*pulsesPerMeter;
positionChange[1] = -(velocity)*deltat*pulsesPerMeter;
}
else if(t<66.64712499999999){
positionChange[0] = (velocity)*deltat*pulsesPerMeter;
positionChange[1] = -(velocity)*deltat*pulsesPerMeter;
}
else if(t<69.77212499999999){
positionChange[0] = (velocity)*deltat*pulsesPerMeter;
positionChange[1] = (velocity)*deltat*pulsesPerMeter;
}
else if(t<70.09609999999999){
positionChange[0] = -(velocity)*deltat*pulsesPerMeter;
positionChange[1] = (velocity)*deltat*pulsesPerMeter;
}
else if(t<79.47109999999999){
positionChange[0] = (velocity)*deltat*pulsesPerMeter;
positionChange[1] = (velocity)*deltat*pulsesPerMeter;
}
else if(t<79.795075){
positionChange[0] = (velocity)*deltat*pulsesPerMeter;
positionChange[1] = -(velocity)*deltat*pulsesPerMeter;
}
else if(t<85.64507499999999){
positionChange[0] = (velocity)*deltat*pulsesPerMeter;
positionChange[1] = (velocity)*deltat*pulsesPerMeter;
}
else{}




  target_f[0] = target_f[0] + positionChange[0];
  target_f[1] = target_f[1] + positionChange[1];
    
    target[1] = (long) target_f[1];
    target[0] = (long) -target_f[0];
  
  
  
}

void setup() {
  Serial.begin(9600);

  for(int k = 0; k < NMOTORS; k++){
    pinMode(enca[k],INPUT);
    pinMode(encb[k],INPUT);
    pinMode(pwm[k],OUTPUT);
    pinMode(in1[k],OUTPUT);


    
  }
  pid[0].setParams(5,0.5,0,255);
  pid[1].setParams(4.3,0.5,0,255);
  attachInterrupt(digitalPinToInterrupt(enca[0]),readEncoder<0>,RISING);
  attachInterrupt(digitalPinToInterrupt(enca[1]),readEncoder<1>,RISING);
  
  Serial.println("target pos");
}

void loop() {

        
  // set target position
  //int target[NMOTORS];
  //target[0] = -200*(prevT/1e6);
  //target[1] = 200*(prevT/1e6);

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  //yo
  setTarget(currT/1.0e6,deltaT);

  // Read the position in an atomic block to avoid a potential misread
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
    pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);
    // signal the motor
    setMotor(dir,pwr,pwm[k],in1[k]);
  }

  for(int k = 0; k < NMOTORS; k++){
    //Serial.print("Target: ");
    //Serial.print(target[k]);
    //Serial.print(" Real_Pos:");
    //Serial.print(pos[k]);
    //Serial.print(" Error:");
    //Serial.print(abs(target[k]-pos[k]));
    //Serial.print(" ");
  }
  //Serial.println();








}

void setMotor(int dir, int pwmVal, int pwm, int in1){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);


  }
  else if(dir == -1){
    digitalWrite(in1,LOW);

  }
  else{

  }  
}

template <int j>
void readEncoder(){
  int b = digitalRead(encb[j]);
  if(b > 0){
    posi[j]++;
  }
  else{
    posi[j]--;
  }
}
