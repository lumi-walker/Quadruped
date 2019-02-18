#include <Cache.h>
#include <Derivative.h>
#include <MsTimer2.h>

#define MA1 7
#define MA2 6
#define STDBY 8
#define PWMA 9
#define encA 2 
//#define encA2 A4 //gray
#define encA2 A5 // purple
double desired_theta_degrees = 100;
double desired_theta = desired_theta_degrees*PI/180;
double dtheta_sensor = 30*(desired_theta); // rad
int16_t ref_tick = int16_t(dtheta_sensor * 12/(2*PI));
int16_t tick_desired = 0;

volatile int16_t cntA = 0;
volatile int16_t cntA_prev = 0;
volatile double err;
volatile double derr = 0;
volatile double derr_prev = 0;

volatile int16_t pwm;
double Ts_ms = 5;
double Ts = Ts_ms/1000.0; // sec

//MA
/*
double KuA = 335;
double TuA = 0.15 / 2; // 2 cycles in 0.15 seconds
double KpA = 0.8*Ku;
double TdA = Tu/8;
*/

//MB
double KuB = 330;
double TuB = 0.08 / 2; // 2 cycles in 0.15 seconds
double KpB = 0.8*KuB;
double TdB = TuB/8;

//void increA() {
//  if(!digitalRead(encA2)) {
//    cntA++;
//  } else {
//    cntA--;
//  }
//}

//MB
void increA() {
  if(digitalRead(encA2)) {
    cntA++;
  } else {
    cntA--;
  }
}

// MA
//void drive(int pwm) {
//
//  
//  bool P1,P2;
//
//  if(pwm > 0) { // rotate forward
//    P1 = true;
//    P2 = false;
//  } else {
//    P1 = false;
//    P2 = true;
//    pwm = pwm*-1;
//    
//  }
//
//  if(pwm < 0) pwm = 0;
//  if(pwm > 255) pwm = 255;
//  
//  digitalWrite(MA1,P1);
//  digitalWrite(MA2,P2);
//  analogWrite(PWMA,pwm);
//}


// MB
void drive(int pwm) {

  
  bool P1,P2;

  if(pwm > 0) { // rotate forward
    P1 = false;
    P2 = true;
  } else {
    P1 = true;
    P2 = false;
    pwm = pwm*-1;
    
  }

  if(pwm < 0) pwm = 0;
  if(pwm > 255) pwm = 255;
  
  digitalWrite(MA1,P1);
  digitalWrite(MA2,P2);
  analogWrite(PWMA,pwm);
}


void do_control() {
  
  err = (tick_desired - cntA);
  derr = (-(cntA - cntA_prev)/Ts + derr_prev)/2;
  derr_prev = derr;
  cntA_prev = cntA;
  
  pwm = KpB*(err + TdB*derr);
  //pwm = Ku*err;
  drive(pwm);
}




uint16_t prevT = 0;
uint16_t dT;
uint16_t elapsedT = 0;
double elapsedT_f = 0;
double timestamp = 0;
uint16_t signalT = 0;
uint16_t DISPLAY_RATE_S = 0.005; // seconds
uint16_t DISPLAY_RATE_uS = DISPLAY_RATE_S*1000000.0;

uint16_t STEP_TIME_S = 2;
uint16_t STEP_TIME_uS = STEP_TIME_S * 1000000;

void setup() {
    digitalWrite(STDBY,HIGH);
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(MA1,OUTPUT);
  pinMode(MA2,OUTPUT);
  pinMode(PWMA,OUTPUT);
  pinMode(encA,INPUT);
  pinMode(encA2,INPUT);
  attachInterrupt(digitalPinToInterrupt(encA),increA,RISING);
  interrupts();

  MsTimer2::set(Ts_ms,do_control);
  //MsTimer2::start();
  prevT = micros();
  //drive(150);
}





void loop() {
  // put your main code here, to run repeatedly:
  dT = micros() - prevT;
  elapsedT += dT;
  timestamp += dT/1000000.0;
  signalT += dT;
  
  prevT = micros();

  if(signalT >= STEP_TIME_S) {
    tick_desired = ref_tick;
  }
  
  if(elapsedT > DISPLAY_RATE_uS) {
    //Serial.println(String(cntA));
    Serial.println(String(timestamp) + "," + String(tick_desired) + "," + String(cntA) + "," + String(pwm));
    elapsedT = 0;
  }
}
