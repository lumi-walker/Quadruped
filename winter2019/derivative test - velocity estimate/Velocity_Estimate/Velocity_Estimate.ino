#include <Cache.h>
#include <Derivative.h>
#include <MsTimer2.h>

#define MA1 7
#define MA2 6
#define STDBY 8
#define PWMA 9
#define encA 2 

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

void runDiff() {
  dcount = cntA - cnt_prev;
  cnt_prev = cntA;

  vel = diff.step(dcount);
}


uint16_t prevT = 0;
uint16_t elapsedT = 0;
uint16_t DISPLAY_RATE_S = 0.5; // seconds
uint16_t DISPLAY_RATE_uS = DISPLAY_RATE_S*1000000.0;

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
  elapsedT += micros() - prevT;
  prevT = micros();

  if(elapsedT > DISPLAY_RATE_uS) {
    Serial.println("velocity : " + String(vel));
    elapsedT = 0;
  }
}
