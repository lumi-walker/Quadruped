#include <Integrator.h>
#include <MsTimer2.h>
#include <Cache.h>
#include <Derivative.h>
#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>
#include <HX711.h>


double Ts_ms = 5;
double Ts_s = Ts_ms * 1000.00;


// MOTOR START*******************************************
#define MA1 7
#define MA2 6
#define MB1 13
#define MB2 12
#define PWMA 9
#define PWMB 10
#define STBY 8

  //encoder interrupt pins
#define encA 2 
#define encB 4 

  // variables storing encoder counts
volatile uint16_t cntA = 0;
volatile uint16_t cntA_prev = 0;
volatile uint16_t dcntA;

volatile uint16_t cntB = 0;
volatile uint16_t cntB_prev = 0;
volatile uint16_t dcntB;

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
void estimateVelocity();
void estimateAcceleration();

  // set pin direction of motor pins
void init_motor_pins();

  // set motor speed and direction
void run_motors(int IN1, int IN2, int PWM, int speed, int direction);

float pwm;

// DATA LOGGING START*******************************************

struct {
    uint16_t countA;
    uint16_t countB;
    float force;
    double timestamp;
} typedef datlog_s;

datlog_s ds;

void update_time(unsigned long& time_elapsed, unsigned long& prev_time );
void log_data(datlog_s& ds, const unsigned long& timestamp);
void print_data_formatted(const datlog_s& ds);
void print_data_raw(datlog_s ds);

unsigned long elapsed_time;
unsigned long prev_time;
unsigned long CONV_FACTOR = 1000000; // us / s


// LOAD CELL START*******************************************  
  // serial clock
#define SCK 5 
  // serial data
#define SDA 3  

  // factor which scales force readings to correct value
float calibration_factor = -185000.51;

  // variable which holds force measurement value
float force;

  // instantiate HX711 object which is responsible for measuring force input
HX711 scale(SDA,SCK);


// CONTROL ************************************************

Integrator acc_int(Ts_s);
Integrator vel_int(Ts_s);



  // mass of the device
double M;
  // desired mass of the device
double Md;
  // desired damping ratio
double Bd;

  // pose setpoint
double acc_sp;
double vel_sp;
double pos_sp;

  // pose estimation from encoder cntA,B and cntA,B_dot
double acc;
double vel;
double pos;

  // position PD controller
double Kp;
double Kd;

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
  scale.tare();
    // set calib factor
  scale.set_scale(calibration_factor);

  init_motor_pins();
  prev_time = micros();

    // run controller ever Ts_ms milliseconds
  MsTimer2::set(Ts_ms,do_control);
  MsTimer2::start();
}



void loop() {
  update_time(elapsed_time, prev_time);

  // measure force from load cell in pounds
  //force = scale.get_units();

  // DO FILTERING

  // DO CONTROL


  // log data
  log_data(ds,elapsed_time);
  print_data_raw(ds);
  
}



// MOTOR START*******************************************
  // increments the encoder counts for motors A and B
void increA() {
  cntA ++;
}
void increB() {
  cntB ++;
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
  pinMode(encA,INPUT);
  pinMode(encB,INPUT);
  attachInterrupt(digitalPinToInterrupt(encA),increA,RISING);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(encB),increB,RISING);
  interrupts();
}

  // set motor speed and direction
void run_motors(int IN1, int IN2, int PWM, int speed, int direction) {
  boolean Pin1 = LOW;
  boolean Pin2 = HIGH;

  if(direction == 1) {
    Pin1 = HIGH;
    Pin2 = LOW;
  } 

  digitalWrite(IN1,Pin1);
  digitalWrite(IN2,Pin2);
  analogWrite(PWM,speed);
}

// DATA LOGGING START*******************************************

void update_time(unsigned long& time_elapsed, unsigned long& prev_time ) {
  time_elapsed += micros() - prev_time;
  prev_time = micros();
}

void log_data(datlog_s& ds,const unsigned long& timestamp) {
  ds.timestamp = (double)(timestamp)/(double)(CONV_FACTOR);
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

// CONTROL START ************************************************

  // estimate velocity and accel
void estimateVelocity() {
  dcntA = cntA - cntA_prev;
  cntA_prev = cntA;

  dcntB = cntB - cntB_prev;
  cntB_prev = cntB;

  cntA_dot = diffA.step(dcntA);
  cntB_dot = diffB.step(dcntB);  
}

void estimateAcceleration() {
  dcntA_dot = cntA_dot - cntA_dot_prev;
  cntA_dot_prev = cntA_dot;

  dcntB_dot = cntB_dot - cntB_dot_prev;
  cntB_dot_prev = cntB_dot;

  cntA_acc = diffA.step(dcntA_dot);
  cntB_acc = diffB.step(dcntB_dot);  
}

  // determine pose setpoint from force measurement
void do_admittance_control() {
  ready_to_read = scale.is_ready();

  if(ready_to_read) {
    force = scale.get_units();

    acc_sp = (force + Bd*vel_sp)/Md;
  }

  vel_sp = acc_int.step(acc_sp);
  pos_sp = vel_int.step(vel_sp);
  
}

  // apply PD position controller
void do_position_control() {
  pwm = Kp * ( pos_sp - pos  ) + Kd * (vel_sp - vel);

  run_motors(MA1, MA2, PWMA, pwm, 1);
  run_motors(MB1, MB2, PWMB, pwm, 1);
}

void do_control() {
  // estimate device pose

  do_admittance_control();
  do_position_control();
}

