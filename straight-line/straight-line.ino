const int enca[] = {3,2};
const int encb[] = {5,4};
const int pwm[] = {10,11};
const int in1[] = {12,13};
int enca_prev[] = {0,0};

long prevT = 0;
volatile int posi[] = {0,0};

#define NMOTORS 2

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