#include <Integrator.h>
#include <MsTimer2.h>
#include <Cache.h>
#include <Derivative.h>
#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>
#include <HX711.h>


// SAMPLING CONFIG
double Ts_ms = 10;
double Ts_s = Ts_ms / 1000.00;
double dt_s;
// DEVICE PARAMS
double WHEEL_RADIUS = 0.032; // meters
double GEAR_RATIO = 1 / 30.00;
double CPR = 12; // counts per rev
double COUNT_TO_LINEAR = (1 / CPR) * (GEAR_RATIO) * (2 * PI) * WHEEL_RADIUS;

// DATA LOGGING **********************************8*

struct {
  int16_t countA;
  int16_t countB;

  double countA_vel;
  double countB_vel;

  double countA_acc;
  double countB_acc;


  double pos_sp; //set point
  double vel_sp;
  double acc_sp;

  float pwmA;
  float pwmB;

  float force;
  double timestamp;
} typedef datlog_s;

datlog_s ds;

unsigned long elapsed_time;
double elapsed_time_s = elapsed_time;
unsigned long prev_time;
unsigned long CONV_FACTOR = 1000000; // us / s


void update_time(unsigned long& time_elapsed, unsigned long& prev_time );
void log_data(datlog_s& ds, const unsigned long& timestamp);
void print_data_formatted(const datlog_s& ds);
void print_data_raw(datlog_s ds);


// MOTOR START*******************************************
#define MA1 7
#define MA2 6
#define MB1 13
#define MB2 12
#define PWMA 9
#define PWMB 10
#define STBY 8

//encoder interrupt pins
#define encA1 2
#define encA2 A4 // grey
#define encB1 4
#define encB2 A5 // purple

// variables storing encoder counts
volatile int16_t cntA = 0;
volatile int16_t cntA_prev = 0;
volatile int16_t dcntA;

volatile int16_t cntB = 0;
volatile int16_t cntB_prev = 0;
volatile int16_t dcntB;

volatile double cntA_dot = 0;
volatile double cntA_dot_prev = 0;
volatile double dcntA_dot;

volatile double cntB_dot = 0;
volatile double cntB_dot_prev = 0;
volatile double dcntB_dot;

volatile double cntA_acc = 0;
volatile double cntB_acc = 0;

uint8_t filter_window_width = 3;

// derivative blocks
Derivative diffA(Ts_s, filter_window_width);
Derivative diffB(Ts_s, filter_window_width);

// increments the encoder counts for motors A and B
void increA();
void increB();

// estimate velocity and accel
void estimatePose();
void estimateVelocity();
void estimateAcceleration();

// set pin direction of motor pins
void init_motor_pins();

// set motor speed and direction
void run_motorA(int PWM);
void run_motorB(int PWM);

float pwmA;
float pwmB;

// LOAD CELL START*******************************************
// serial clock
#define SCK A3 // yellow
// serial data
#define SDA A2 // orange

// factor which scales force readings to correct value
float calibration_factor = -185000.51;

// variable which holds force measurement value
float force;

// instantiate HX711 object which is responsible for measuring force input
HX711 scale(SDA, SCK);


// CONTROL ************************************************

volatile double prevt;
volatile double dt;
Integrator acc_int(Ts_s);
Integrator vel_int(Ts_s);

// desired mass of the device
double Md = 2; // kg
// desired damping ratio
double Bd = 4;

double ALPHA = 1 / COUNT_TO_LINEAR;

// pose setpoint
double acc_sp = 0;
double vel_sp = 0;
double pos_sp = 0;

// pose estimation from encoder cntA,B and cntA,B_dot
double accA;
double velA;
double posA;

double accB;
double velB;
double posB;

// found this at Ts = 10ms using Ziegler-Nichols tuning method
// position PD controller
double Kp_A = 0.8 * 2095;
double Kp_B = 0.8 * 2040;

double Td_A = 0.167 / 8 ;
double Td_B = 0.1625 / 8 ;

// flag indicating new force measurement ready
bool ready_to_read = false;

// determine pose setpoint from force measurement
void do_admittance_control();

// apply PD position controller
void do_position_control();

void do_control();

void setup() {

  Serial.begin(9600);
  // zero out load cell
  pinMode(SDA, INPUT);
  pinMode(SCK, OUTPUT);
  scale.tare();
  // set calib factor
  scale.set_scale(calibration_factor);

  init_motor_pins();
  prev_time = micros();

  // run controller ever Ts_ms milliseconds
  MsTimer2::set(Ts_ms, do_control);
  MsTimer2::start();
  prevt = micros();
  //pos_sp = 0.1/COUNT_TO_LINEAR;
//  Serial.println("Md = " + String(Md) + "," + "Bd = " + String(Bd) + "," );
}


void loop() {
  update_time(elapsed_time, prev_time);
  //  elapsed_time_s = double(elapsed_time / 100000);
  // measure force from load cell in pounds
  //  force = scale.get_units();

  // DO FILTERING

  // DO CONTROL

  do_control();
  // log data
  //log_data(ds,elapsed_time);
  //print_data_raw(ds);

  //Serial.println(String(elapsed_time_s) + "," + String(force) + "," + String(acc_sp ) + "," + String(vel_sp )+ "," + String(pos_sp));
  Serial.println(String(elapsed_time) + "," + String(force) + "," + String(pos_sp ) + "," + String(vel_sp ) + "," + String(acc_sp) + "," + String(posA ) + "," + String(velA ) + "," + String(accA)+ "," + String(pwmA));

}



// MOTOR START*******************************************
// increments the encoder counts for motors A and B
void increA() {
  if (digitalRead(encA2)) {
    cntA ++;
  } else {
    cntA--;
  }
}
void increB() {
  if (!digitalRead(encB2)) {
    cntB ++;
  } else {
    cntB--;
  }
}

// set pin direction of motor pins
void init_motor_pins() {
  //Control the direction of motor 2, 01 is positive, 10 is reverse.
  pinMode(MA1, OUTPUT);
  pinMode(MA2, OUTPUT);
  pinMode(MB1, OUTPUT);
  pinMode(MB2, OUTPUT);
  pinMode(PWMA, OUTPUT);//PWM of left motor
  pinMode(PWMB, OUTPUT);//PWM of right motor
  pinMode(STBY, OUTPUT);//enable TB6612FNG
  pinMode(encA1, INPUT);
  pinMode(encA2, INPUT);
  pinMode(encB1, INPUT);
  pinMode(encB2, INPUT);
  digitalWrite(STBY, HIGH);
  attachInterrupt(digitalPinToInterrupt(encA1), increA, RISING);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(encB1), increB, RISING);
  interrupts();
}

// set motor speed and direction
void run_motorA(int PWM) {

  if (PWM < 0) {
    digitalWrite(MA1, HIGH);
    digitalWrite(MA2, LOW);
    PWM *= -1;
  } else {
    digitalWrite(MA1, LOW);
    digitalWrite(MA2, HIGH);

  }

  if (PWM < 0) PWM = 0;
  if (PWM > 255) PWM = 255;

  analogWrite(PWMA, PWM);
}

void run_motorB(int PWM) {

  if (PWM < 0) {
    digitalWrite(MB1, HIGH);
    digitalWrite(MB2, LOW);
    PWM *= -1;
  } else {
    digitalWrite(MB1, LOW);
    digitalWrite(MB2, HIGH);

  }

  if (PWM < 0) PWM = 0;
  if (PWM > 255) PWM = 255;

  analogWrite(PWMB, PWM);
}

// CONTROL START ************************************************

// estimate velocity and accel
void estimateVelocity() {
  dcntA = cntA - cntA_prev;
  cntA_prev = cntA;

  dcntB = cntB - cntB_prev;
  cntB_prev = cntB;

  cntA_dot = diffA.step(dcntA);
  cntB_dot = diffB.step(dcntB);

  velA = cntA_dot * COUNT_TO_LINEAR;
  velB = cntB_dot * COUNT_TO_LINEAR;
}

void estimateAcceleration() {
  dcntA_dot = cntA_dot - cntA_dot_prev;
  cntA_dot_prev = cntA_dot;

  dcntB_dot = cntB_dot - cntB_dot_prev;
  cntB_dot_prev = cntB_dot;

  cntA_acc = diffA.step(dcntA_dot);
  cntB_acc = diffB.step(dcntB_dot);
  accA = cntA_acc * COUNT_TO_LINEAR;
  accB = cntB_acc * COUNT_TO_LINEAR;
}



// determine pose setpoint from force measurement
void do_admittance_control() {
  ready_to_read = scale.is_ready();

  if (ready_to_read) {

//    force = 4.44822 * scale.get_units(); //convert lbf to N
    force = 3.5 * scale.get_units(); //convert lbf to N
    if (force <= 0.45) {
      force = 0;
      acc_sp = 0;
      vel_sp = 0;
    }
    //acc_sp = (force + Bd*vel_sp)/Md;
    acc_sp = (force - Bd * vel_sp) / Md; // acceleration of encoder counts
    
    //max acc 40
    if (acc_sp >= 40) {
      acc_sp = 40;
    }
    else if (acc_sp <= -40) {
      acc_sp = -40;
    }
    dt = (micros()-prevt)/1000000.00;
    prevt = micros();
    vel_sp = acc_int.stept(acc_sp,dt);
    //vel_sp = acc_int.step(acc_sp); // velocity of encoder counts

    if(vel_sp >= .7) vel_sp = .7;
    
    if (force <= 0.45) {
      force = 0;
      acc_sp = 0;
      vel_sp = 0;
    }
    //pos_sp = vel_int.step(vel_sp); // position (number) of encoder counts
    pos_sp = acc_int.stept(vel_sp,dt);
    //  vel_sp *= ALPHA;
    //  pos_sp *= ALPHA;
  }



}

// apply PD position controller
void do_position_control() {
  estimateVelocity();
//    estimateAcceleration();
  posA = cntA * COUNT_TO_LINEAR;
  posB = cntB * COUNT_TO_LINEAR;
  //  pwmA = Kp_A *( ( pos_sp - cntA  ) + Td_A * (vel_sp - cntA_dot));
  //  pwmB = Kp_B *( ( pos_sp - cntB  ) + Td_B * (vel_sp - cntB_dot));
  pwmA = Kp_A * ( ( pos_sp - posA  ) + Td_A * (vel_sp - velA));
  pwmB = Kp_B * ( ( pos_sp - posB  ) + Td_B * (vel_sp - velB));
 
  run_motorA(pwmA);
  run_motorB(pwmB);
//  run_motorA(0);
//  run_motorB(0);
  
}

void do_control() {
  // estimate device pose
  do_admittance_control();
  do_position_control();
}


// DATA LOGGING START*******************************************


void update_time(unsigned long& time_elapsed, unsigned long& prev_time ) {
  time_elapsed += micros() - prev_time;
  dt_s = (micros() - prev_time)/100000;
  prev_time = micros();
}

void log_data(datlog_s& ds, const unsigned long& timestamp) {
  ds.timestamp = (double)(timestamp) / (double)(CONV_FACTOR);
  ds.countA = cntA;
  ds.countB = cntB;
  ds.force = force;
}

void print_data_formatted(const datlog_s& ds) {
  Serial.print("timestamp: " + String(ds.timestamp) + " // ");
  Serial.print("(countA,countB) : (" + String(ds.countA) + "," + String(ds.countB) + ") // ");
  Serial.println("force : " + String(ds.force));
}

void print_data_raw(datlog_s ds) {
  Serial.println(String(ds.timestamp) + "," + String(ds.countA) + "," + String(ds.countB) + "," + String(ds.force));
}
