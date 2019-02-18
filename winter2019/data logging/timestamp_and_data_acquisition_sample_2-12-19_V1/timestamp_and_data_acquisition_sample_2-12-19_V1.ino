#include <PinChangeInterrupt.h>
#include <PinChangeInterruptBoards.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptSettings.h>

#include <HX711.h>

#define MA1 7
#define MA2 6
#define MB1 13
#define MB2 12
#define PWMA 9
#define PWMB 10
#define STBY 8

#define encA 2 //interrupt 0
#define encB 4 // interrupt 1

#define SCK A3 // serial clock
#define SDA  A2 // serial data

HX711 scale(SDA,SCK);
float calibration_factor = -185000.51;

struct data_s {
  uint16_t tickA;
  uint16_t tickB;
  float force;
  unsigned long timestamp;
}typedef Data_s;

volatile uint16_t tickA = 0;
volatile uint16_t tickB = 0;
volatile unsigned long prevT;
volatile unsigned long elapsedT;

void tick_encA() {
  tickA++;
}

void tick_encB() {
  tickB++;
}

void print_formatted(const Data_s& ds) {
  Serial.println("timestamp: " + String(ds.timestamp) + " // " + "(countA,countB): (" + String(ds.tickA) + "," + String(ds.tickB) + ") // force : " + String(ds.force));
}

void print_raw(const Data_s& ds) {
  Serial.println(String(ds.timestamp) + "," + String(ds.tickA) + "," + String(ds.tickB) + "," + String(ds.force));
}
void runset(int motor, int speed, int direction){  
  
  digitalWrite(STBY, HIGH); //EN PIN
  
  boolean Pin1 = LOW;  
  boolean Pin2 = HIGH;  
  
  if(direction == 1){  
    Pin1 = HIGH;  
    Pin2 = LOW;  
  }  
  
  if(motor == 1){  
    digitalWrite(MA1, Pin1);  
    digitalWrite(MA2, Pin2);  
    analogWrite(PWMA, speed);  
  }else{  
    digitalWrite(MB1, Pin1);  
    digitalWrite(MB2, Pin2);  
    analogWrite(PWMB, speed);  
  }  
}  


void init_motor_pins() {
  pinMode(MA1, OUTPUT);//Control the direction of motor 1, 01 is positive, 10 is reverse.
  pinMode(MA2, OUTPUT);
  pinMode(MB1, OUTPUT);//Control the direction of motor 2, 01 is positive, 10 is reverse.
  pinMode(MB2, OUTPUT);
  pinMode(PWMA, OUTPUT);//PWM of left motor
  pinMode(PWMB, OUTPUT);//PWM of right motor
  pinMode(STBY, OUTPUT);//enable TB6612FNG
  pinMode(encA,INPUT);
  pinMode(encB,INPUT);
  attachInterrupt(digitalPinToInterrupt(encA),tick_encA,RISING);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(encB),tick_encB,RISING);
  interrupts();
}

int motor1 = 1;
int motor2 = 2;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  scale.tare();
  scale.set_scale(calibration_factor);

  prevT = micros();
  init_motor_pins();
}


float force;
float Kp = 150;
float pwm;
bool halted = true;

float min_pwm = 110;
float max_pwm = 150;
float min_force = 0.1;
float max_force = 0.6;

float rescale(float force) {
  float pwm;
  
  if(force < min_force) pwm = min_force;
  if(force > max_force) pwm = max_force;
  pwm -= min_force;
  pwm /= (max_force - min_force);
  pwm *- (max_pwm - min_pwm);
  pwm += min_pwm;
  return pwm;
  
}

Data_s ds;

void loop() {
  // put your main code here, to run repeatedly:
  elapsedT += micros() - prevT;
  prevT = micros();
  
  force = scale.get_units();
  //force = 0;
  
  ds.force = force;
  ds.tickB = tickB;
  ds.tickA = tickA;
  ds.timestamp = elapsedT;
  
  if(force > 0.1) { 
    pwm = rescale(force);

    if(halted) halted = false;
    runset(motor1,pwm,1);
    runset(motor2,pwm,1);
    
  } else {
    if(!halted) {
      runset(motor1,0,1);
      runset(motor2,0,1);
      halted = true;
    }
    
  }

  print_raw(ds);

  
}
