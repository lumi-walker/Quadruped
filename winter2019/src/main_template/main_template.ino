#include <HX711.h>

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
volatile uint16_t cntB = 0;
  // increments the encoder counts for motors A and B
void increA();
void increB();
  // set pin direction of motor pins
void init_motor_pins();
  // set motor speed and direction
void run_motors(int IN1, int IN2, int PWM, int speed, int direction);

float pwm;
bool halted = true;
// MOTOR END***************************************************
//**************************************************************
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

// DATA LOGGING END*******************************************

//**************************************************************

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

// LOAD CELL END*******************************************

void setup() {
  
  Serial.begin(9600);
    // zero out load cell
  scale.tare();
    // set calib factor
  scale.set_scale(calibration_factor);

  init_motor_pins();
  prev_time = micros();
}

void loop() {
  update_time(elapsed_time, prev_time);

  // measure force from load cell in pounds
  force = scale.get_units();

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
  attachInterrupt(digitalPinToInterrupt(encB),increB,RISING);
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
// MOTOR END*******************************************
//**************************************************************
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
// DATA LOGGING END*******************************************
