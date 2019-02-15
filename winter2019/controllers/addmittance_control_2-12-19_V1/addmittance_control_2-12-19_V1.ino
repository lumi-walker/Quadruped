#include <HX711.h>

// define motor driver pins
#define MA1 7
#define MA2 6
#define MB1 13
#define MB2 12
#define PWMA 9
#define PWMB 10
#define STBY 8

// define encoder interrupt pins
#define encA 2 //interrupt 0
#define encB 4 // interrupt 1

// define load cell I2C pins
#define SCK 5 // serial clock
#define SDA 3  // serial data

// drives motor <motor> in the direction <direction> at speed <speed>
void runset(int motor, int speed, int direction);

// sets direction of motor pins and attach interrupts
void init_motor_pins();


float min_pwm = 110;
float max_pwm = 150;
float min_force = 0.1;
float max_force = 0.6;

// linearly maps force to pwm, (min_force,max_force) -> (min_pwm,max_pwm)
float rescale(float force);


int motor1 = 1;
int motor2 = 2;

HX711 scale(SDA,SCK);
float calibration_factor = -185000.51;

// force measurement from HX711 in lbs
float force;

// pwm to motors
float pwm;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  init_motor_pins();
  scale.tare();
  scale.set_scale(calibration_factor);
}


void loop() {
  // put your main code here, to run repeatedly:
  force = scale.get_units();
  if(force > 0.1) { 
    pwm = rescale(force);

  } else {
    pwm = 0;
  }
    runset(motor1,pwm,1);
    runset(motor2,pwm,1); 
}


void init_motor_pins() {
  pinMode(MA1, OUTPUT);//Control the direction of motor 1, 01 is positive, 10 is reverse.
  pinMode(MA2, OUTPUT);
  pinMode(MB1, OUTPUT);//Control the direction of motor 2, 01 is positive, 10 is reverse.
  pinMode(MB2, OUTPUT);
  pinMode(PWMA, OUTPUT);//PWM of left motor
  pinMode(PWMB, OUTPUT);//PWM of right motor
  pinMode(STBY, OUTPUT);//enable TB6612FNG
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
