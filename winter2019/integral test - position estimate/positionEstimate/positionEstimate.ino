#include <Integrator.h>

#include <Cache.h>
#include <Derivative.h>
#include <MsTimer2.h>

#define MA1 7
#define MA2 6
#define STDBY 8
#define PWMA 9
#define encA 2 

uint8_t sinLookUp[200] = {25,26,27,27,28,29,30,30,31,32,
33,33,34,35,36,36,37,38,38,39,
40,40,41,42,42,43,43,44,44,45,
45,46,46,47,47,47,48,48,48,49,
49,49,49,49,50,50,50,50,50,50,
50,50,50,50,50,50,50,49,49,49,
49,49,48,48,48,47,47,47,46,46,
45,45,44,44,43,43,42,42,41,40,
40,39,38,38,37,36,36,35,34,33,
33,32,31,30,30,29,28,27,27,26,
25,24,23,23,22,21,20,20,19,18,
17,17,16,15,14,14,13,12,12,11,
10,10,9,8,8,7,7,6,6,5,
5,4,4,3,3,3,2,2,2,1,
1,1,1,1,0,0,0,0,0,0,
0,0,0,0,0,0,0,1,1,1,
1,1,2,2,2,3,3,3,4,4,
5,5,6,6,7,7,8,8,9,10,
10,11,12,12,13,14,14,15,16,17,
17,18,19,20,20,21,22,23,23,24,};

uint8_t basePWM = 150;

volatile double cntA_estimate = 0;

volatile uint16_t cntA = 0;
volatile uint16_t cnt_capture;
volatile uint16_t cnt_prev = 0;
volatile double vel;

uint8_t window_width = 3;
double Ts_ms = 5;
double Ts = Ts_ms/1000.0; // sec
uint16_t dcount;
void increA() {
  cntA++;
}


void drive(int pwm) {
  digitalWrite(STDBY,HIGH);
  digitalWrite(MA1,HIGH);
  digitalWrite(MA2,LOW);
  analogWrite(PWMA,pwm);
}

Derivative diff(Ts,window_width);
Integrator integral(Ts);

void runDiff() {
  dcount = cntA - cnt_prev;
  cnt_prev = cntA;

  vel = diff.step(dcount);

  cntA_estimate = integral.step(vel);
}


uint16_t prevT = 0;
uint16_t dT;
uint16_t elapsedT = 0;
uint16_t timestamp = 0;
uint16_t DISPLAY_RATE_S = 0.5; // seconds
uint16_t DISPLAY_RATE_uS = DISPLAY_RATE_S*1000000.0;
uint16_t SIN_STEP_mS = 50;
uint16_t SIN_STEP_uS = SIN_STEP_mS * 1000;
uint16_t sin_step_trig = 0;
uint16_t iter = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(MA1,OUTPUT);
  pinMode(MA2,OUTPUT);
  pinMode(PWMA,OUTPUT);
  pinMode(encA,INPUT);
  attachInterrupt(digitalPinToInterrupt(encA),increA,RISING);
  interrupts();

  MsTimer2::set(Ts_ms,runDiff);
  MsTimer2::start();
  prevT = micros();
}





void loop() {
  // put your main code here, to run repeatedly:
  dT = micros() - prevT;
  elapsedT += dT;
  timestamp += dT;
  sin_step_trig += dT;
  
  prevT = micros();

  if(sin_step_trig >= SIN_STEP_uS) {
    iter = iter%200;
    iter++;
    drive(sinLookUp[iter] + basePWM);
    sin_step_trig = 0;
  }
  
  if(elapsedT > DISPLAY_RATE_uS) {
    Serial.println(String(timestamp) +  "," + String(sinLookUp[iter]) + "," + String(cntA) + "," + String(vel) + "," + String(cntA_estimate));
    elapsedT = 0;
  }
}
