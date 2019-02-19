#include <Cache.h>
#include <Derivative.h>
#include <MsTimer2.h>
#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>

#define MA1 7
#define MA2 6
#define MB1 13
#define MB2 12
#define PWMA 9
#define PWMB 10
#define STDBY 8

#define encA1 2
#define encA2 A4 // grey (A)
#define encB1 4
#define encB2 A5 // purple



double WHEEL_RADIUS = 0.032; // meters
double GEAR_RATIO = 1 / 30.00;
double CPR = 12; // counts per rev
double COUNT_TO_LINEAR = (1 / CPR) * (GEAR_RATIO) * (2 * PI) * WHEEL_RADIUS;

double desired_theta_degrees = 100;
double desired_theta = desired_theta_degrees * PI / 180;
//double dtheta_sensor = 30*(desired_theta); // rad
//int16_t ref_tick = int16_t(dtheta_sensor * 12/(2*PI));
//int16_t tick_desired = 0;

double ref_x = desired_theta * WHEEL_RADIUS; //meters
double x_desired = 0;

volatile int16_t cnt = 0;
volatile int16_t cnt_prev = 0;
volatile double dist = 0;
volatile double dist_prev = 0;
volatile double err;
volatile double derr = 0;
volatile double derr_prev = 0;

volatile int16_t pwm;
double Ts_ms = 10;
double Ts = Ts_ms / 1000.0; // sec

//MA

double KuA = 27950;
double TuA = 0.167; // 2 cycles in 0.15 seconds
double KpA = 0.8 * KuA;
double TdA = TuA / 8;


//MB
double KuB = 27400;
double TuB = 0.1625;
double KpB = 0.8 * KuB;
double TdB = TuB / 8;

//MA
void incre() {
  if (!digitalRead(encA2)) {
    cnt++;
    dist = cnt * COUNT_TO_LINEAR;
  } else {
    cnt--;
    dist = cnt * COUNT_TO_LINEAR;
  }
}

//MB
//void incre() {
//  if (!digitalRead(encB2)) {
//    cnt++;
//    dist = cnt * COUNT_TO_LINEAR;
//  } else {
//    cnt--;
//    dist = cnt * COUNT_TO_LINEAR;
//  }
//}

// MA
void drive(int pwm) {


  bool P1, P2;

  if (pwm > 0) { // rotate forward
    P1 = true;
    P2 = false;
  } else {
    P1 = false;
    P2 = true;
    pwm = pwm * -1;

  }

  if (pwm < 0) pwm = 0;
  if (pwm > 255) pwm = 255;

  digitalWrite(MA1, P1);
  digitalWrite(MA2, P2);
  analogWrite(PWMA, pwm);
}


// MB
//void drive(int pwm) {
//
//
//  bool P1, P2;
//
//  if (pwm > 0) { // rotate forward
//    P1 = false;
//    P2 = true;
//  } else {
//    P1 = true;
//    P2 = false;
//    pwm = pwm * -1;
//
//  }
//
//  if (pwm < 0) pwm = 0;
//  if (pwm > 255) pwm = 255;
//
//  digitalWrite(MB1, P1);
//  digitalWrite(MB2, P2);
//  analogWrite(PWMB, pwm);
//}


void do_control() {
  err = (x_desired - dist);
  derr = (-(dist - dist_prev) / Ts + derr_prev) / 2;
  derr_prev = derr;
  dist_prev = dist;

    pwm = KpA * (err + TdA * derr);
//  pwm = KuA * err;
  drive(pwm);
  //  drive(0);
}




uint16_t prevT = 0;
uint16_t dT;
uint16_t elapsedT = 0;
double elapsedT_f = 0;
double timestamp = 0;
uint16_t signalT = 0;
uint16_t DISPLAY_RATE_S = 0.005; // seconds
uint16_t DISPLAY_RATE_uS = DISPLAY_RATE_S * 1000000.0;

uint16_t STEP_TIME_S = 2;
uint16_t STEP_TIME_uS = STEP_TIME_S * 1000000;

void setup() {
  digitalWrite(STDBY, HIGH);
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(MA1, OUTPUT);
  pinMode(MA2, OUTPUT);
  pinMode(MB1, OUTPUT);
  pinMode(MB2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  //MA
  pinMode(encA1, INPUT);
  pinMode(encA2, INPUT);
  attachInterrupt(digitalPinToInterrupt(encA1), incre, RISING);

  //MB
  //  pinMode(encB1, INPUT);
  //  pinMode(encB2, INPUT);
  //  //  attachInterrupt(digitalPinToInterrupt(encB1), incre, RISING);
  //  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(encB1), incre, RISING);

  interrupts();

  MsTimer2::set(Ts_ms, do_control);
  //  MsTimer2::start();
  prevT = micros();
  //  drive(0);
}





void loop() {
  // put your main code here, to run repeatedly:
  dT = micros() - prevT;
  elapsedT += dT;
  timestamp += dT / 1000000.0;
  signalT += dT;

  prevT = micros();

  if (signalT >= STEP_TIME_S) {
    //    tick_desired = ref_tick;
    x_desired = ref_x;
  }
  do_control();
  if (elapsedT > DISPLAY_RATE_uS) {
    //Serial.println(String(cntA));
    //    Serial.println(String(timestamp) + "," + String(x_desired) + "," + String(dist) + "," + String(pwm));
    Serial.println(String(timestamp) + "," + String(dist) + "," + String(pwm));
    elapsedT = 0;
  }
}
