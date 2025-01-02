
#include <util/atomic.h>


float P = 6;
float MAX_P = 7.1;

float calc_velocity = 0;

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
  void evalu(int value, int target,float velocity, float targetVel, float deltaT, int &pwr, int &dir){
    // error
    int e = target - value;
    targetVel /= 100;
    float eVel = targetVel - velocity;
    float velocityWeight = 0.5; // Adjust this based on priorities
    float eCombined = e + velocityWeight * eVel;
    // derivative
    float dedt = (e-eprev)/(deltaT);
  
    // integral
    eintegral = eintegral + e*deltaT;
  
    // control signal
    float u = kp*eCombined + kd*dedt + ki*eintegral;
  
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
float velocity[NMOTORS];
float prevPos[NMOTORS];
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
float positionChange[2] = {0, 0};
float g_time;
float g_velocity = 0.16; // m/s
float l_time = 0;
float pulsesPerTurn = 960;
float pulsesPerMeter = pulsesPerTurn*3.9788;
float deltaT;
void setTarget(float t, float deltat){
  
  


  g_time = fmod(t,124); // time is in seconds
  l_time = 0;
   Halt(3);
  Right();
  Halt(3);
   





  target_f[0] = target_f[0] + positionChange[0];
  target_f[1] = target_f[1] + positionChange[1];
    
    target[1] = (long) target_f[1];
    target[0] = (long) -target_f[0];
  
  
  
}

void setup() {
  Serial.begin(57600);

  for(int k = 0; k < NMOTORS; k++){
    pinMode(enca[k],INPUT);
    pinMode(encb[k],INPUT);
    pinMode(pwm[k],OUTPUT);
    pinMode(in1[k],OUTPUT);


    
  }
  pid[0].setParams(5,0.5,0,255);
  pid[1].setParams(5,0.5,0,255);
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
  deltaT = ((float) (currT - prevT))/( 1.0e6 );
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
  
    for (int k = 0; k < NMOTORS; k++) {
        int deltaPos = pos[k] - prevPos[k];
        velocity[k] = deltaPos / deltaT; // Ticks per second
        prevPos[k] = pos[k];        // Update previous position
    }

  for (int k = 0; k < NMOTORS; k++) {
        velocity[k] /= pulsesPerMeter; // Velocity in meters per second
  }  
  // loop through the motors
  //for(int k = 0; k < NMOTORS; k++){
    int pwr, dir;
    // evaluate the control signal
    pid[0].evalu(pos[0],target[0],velocity[0],calc_velocity,deltaT,pwr,dir);
    // signal the motor
    setMotor(dir,pwr,pwm[0],in1[0]);
    // evaluate the control signal
    pid[1].evalu(pos[1],target[1],velocity[1],calc_velocity,deltaT,pwr,dir);
    // signal the motor
    setMotor(dir,pwr,pwm[1],in1[1]);
  //}
  float m0_vel = abs((pos[0]/deltaT));
  float m1_vel = abs((pos[1]/deltaT));
  Serial.print(m0_vel);
  Serial.print("   ");
  Serial.println(m1_vel);
  
  for(int k = 0; k < NMOTORS; k++){
    //Serial.print("Target: ");
    //Serial.print(target[k]);
    //Serial.print(" Error:");
    //Serial.print(abs(target[k]-pos[k]));
    //Serial.print(" ");
  }
  //Serial.print(" Real_Pos(1):");
  //Serial.print(abs(pos[0]));
  //Serial.print(" Real_Pos(2):");
  //Serial.print(abs(pos[1]));
  //Serial.println();

  // if(abs(target[0]-pos[0])+abs(target[1]-pos[1])>5){
  //   if (P < MAX_P){
  //     P+=0.1;
  //   }else{
  //     P = MAX_P;
  //   }
    
  // }






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

void Halt(float time){
  if(g_time<time+l_time && g_time > l_time){
    positionChange[0] = 0;
  positionChange[1] = 0;
  posi[0]=target[0];
  posi[1]=target[1];
  velocity[0]=calc_velocity/100;
   velocity[1]=calc_velocity/100;
  }
  l_time += time;
}




void Forward(float cm){
  float acc_time = 0.4; //in seconds
float velocity = g_velocity*100;
calc_velocity = 0;
float acc = velocity/acc_time;
// Distance = (1/2) * acceleration time * maximum velocity + (constant velocity * constant velocity time) + (1/2) * deceleration time * maximum velocity
// Distance = acceleration time * maximum velocity + (constant velocity * constant velocity time)
// Distance =   (acceleration time * maximum velocity) + (constant velocity * constant velocity time)
//Distance - (acceleration time * maximum velocity) = (constant velocity * constant velocity time)
//(Distance - (acceleration time * maximum velocity cm))/constant velocity cm = constant velocity time
//10 cm / 16 cm/s
float cv_time = (cm/velocity) - acc_time;
float move_time = l_time+(cm/velocity)+acc_time;
if(g_time<l_time+acc_time && g_time > l_time){
  calc_velocity = acc*(g_time - l_time);
}

if(g_time<l_time+acc_time+cv_time && g_time > l_time+acc_time){
  calc_velocity = velocity;
}

if(g_time<l_time+2*acc_time+cv_time && g_time >l_time+acc_time+cv_time){
  calc_velocity = velocity - ((g_time - (l_time + acc_time + cv_time)) * acc);

  //Serial.println(move_time);

}
if(g_time<move_time && g_time > l_time){
  positionChange[0] = (abs(calc_velocity)/100)*deltaT*pulsesPerMeter;
  positionChange[1] = (abs(calc_velocity)/100)*deltaT*pulsesPerMeter;
}

l_time = move_time;

}

void Backward(float cm){
 float acc_time = 0.4; //in seconds
float velocity = g_velocity*100;
calc_velocity = 0;
float acc = velocity/acc_time;
// Distance = (1/2) * acceleration time * maximum velocity + (constant velocity * constant velocity time) + (1/2) * deceleration time * maximum velocity
// Distance = acceleration time * maximum velocity + (constant velocity * constant velocity time)
// Distance =   (acceleration time * maximum velocity) + (constant velocity * constant velocity time)
//Distance - (acceleration time * maximum velocity) = (constant velocity * constant velocity time)
//(Distance - (acceleration time * maximum velocity cm))/constant velocity cm = constant velocity time
//10 cm / 16 cm/s
float cv_time = (cm/velocity) - acc_time;
float move_time = l_time+(cm/velocity)+acc_time;
if(g_time<l_time+acc_time && g_time > l_time){
  calc_velocity = acc*(g_time - l_time);
}

if(g_time<l_time+acc_time+cv_time && g_time > l_time+acc_time){
  calc_velocity = velocity;
}

if(g_time<l_time+2*acc_time+cv_time && g_time >l_time+acc_time+cv_time){
  calc_velocity = velocity - ((g_time - (l_time + acc_time + cv_time)) * acc);

  //Serial.println(move_time);

}
if(g_time<move_time && g_time > l_time){
  positionChange[0] = -(abs(calc_velocity)/100)*deltaT*pulsesPerMeter;
  positionChange[1] = -(abs(calc_velocity)/100)*deltaT*pulsesPerMeter;
}

l_time = move_time;
}

void Left(){

float cm = 5.187;//orignal = 5.184



float acc_time = 0.4; //in seconds
float velocity = g_velocity*100;
calc_velocity = 0;
float acc = velocity/acc_time;
// Distance = (1/2) * acceleration time * maximum velocity + (constant velocity * constant velocity time) + (1/2) * deceleration time * maximum velocity
// Distance = acceleration time * maximum velocity + (constant velocity * constant velocity time)
// Distance =   (acceleration time * maximum velocity) + (constant velocity * constant velocity time)
//Distance - (acceleration time * maximum velocity) = (constant velocity * constant velocity time)
//(Distance - (acceleration time * maximum velocity cm))/constant velocity cm = constant velocity time
//10 cm / 16 cm/s
float cv_time = (cm/velocity) - acc_time;
float move_time = l_time+(cm/velocity)+acc_time;
if(g_time<l_time+acc_time && g_time > l_time){
  calc_velocity = acc*(g_time - l_time);
}

if(g_time<l_time+acc_time+cv_time && g_time > l_time+acc_time){
  calc_velocity = velocity;
}

if(g_time<l_time+2*acc_time+cv_time && g_time >l_time+acc_time+cv_time){
  calc_velocity = velocity - ((g_time - (l_time + acc_time + cv_time)) * acc);

  //Serial.println(move_time);

}
if(g_time<move_time && g_time > l_time){
  positionChange[0] = (abs(calc_velocity)/100)*deltaT*pulsesPerMeter;
  positionChange[1] = -(abs(calc_velocity)/100)*deltaT*pulsesPerMeter;
}

l_time = move_time;
}

void Right(){

  float cm = 5.187;
 


 float acc_time = 0.4; //in seconds
float velocity = g_velocity*100;
calc_velocity = 0;
float acc = velocity/acc_time;
// Distance = (1/2) * acceleration time * maximum velocity + (constant velocity * constant velocity time) + (1/2) * deceleration time * maximum velocity
// Distance = acceleration time * maximum velocity + (constant velocity * constant velocity time)
// Distance =   (acceleration time * maximum velocity) + (constant velocity * constant velocity time)
//Distance - (acceleration time * maximum velocity) = (constant velocity * constant velocity time)
//(Distance - (acceleration time * maximum velocity cm))/constant velocity cm = constant velocity time
//10 cm / 16 cm/s
float cv_time = (cm/velocity) - acc_time;
float move_time = l_time+(cm/velocity)+acc_time;
if(g_time<l_time+acc_time && g_time > l_time){
  calc_velocity = acc*(g_time - l_time);
}

if(g_time<l_time+acc_time+cv_time && g_time > l_time+acc_time){
  calc_velocity = velocity;
}

if(g_time<l_time+2*acc_time+cv_time && g_time >l_time+acc_time+cv_time){
  calc_velocity = velocity - ((g_time - (l_time + acc_time + cv_time)) * acc);

  //Serial.println(move_time);

}
if(g_time<move_time && g_time > l_time){
  positionChange[0] = -(abs(calc_velocity)/100)*deltaT*pulsesPerMeter;
  positionChange[1] = (abs(calc_velocity)/100)*deltaT*pulsesPerMeter;
}

l_time = move_time;

}
