#include <Cache.h>
#include <Derivative.h>
#include <MsTimer2.h>

#define MA1 7
#define MA2 6
#define STDBY 8
#define PWMA 9
#define encA 2 

uint8_t sinLookUp[500] = {128,129,131,132,134,136,137,139,140,142,
143,145,147,148,150,151,153,155,156,158,
159,161,162,164,165,167,168,170,171,173,
174,176,177,179,180,182,183,185,186,188,
189,190,192,193,194,196,197,199,200,201,
202,204,205,206,208,209,210,211,212,214,
215,216,217,218,219,220,222,223,224,225,
226,227,228,229,230,231,232,233,233,234,
235,236,237,238,238,239,240,241,241,242,
243,244,244,245,245,246,247,247,248,248,
249,249,250,250,251,251,251,252,252,252,
253,253,253,254,254,254,254,254,255,255,
255,255,255,255,255,255,255,255,255,255,
255,255,255,254,254,254,254,254,253,253,
253,252,252,252,251,251,251,250,250,249,
249,248,248,247,247,246,245,245,244,244,
243,242,241,241,240,239,238,238,237,236,
235,234,233,233,232,231,230,229,228,227,
226,225,224,223,222,220,219,218,217,216,
215,214,212,211,210,209,208,206,205,204,
202,201,200,199,197,196,194,193,192,190,
189,188,186,185,183,182,180,179,177,176,
174,173,171,170,168,167,165,164,162,161,
159,158,156,155,153,151,150,148,147,145,
143,142,140,139,137,136,134,132,131,129,
128,126,124,123,121,119,118,116,115,113,
112,110,108,107,105,104,102,100,99,97,
96,94,93,91,90,88,87,85,84,82,
81,79,78,76,75,73,72,70,69,67,
66,65,63,62,61,59,58,56,55,54,
53,51,50,49,47,46,45,44,43,41,
40,39,38,37,36,35,33,32,31,30,
29,28,27,26,25,24,23,22,22,21,
20,19,18,17,17,16,15,14,14,13,
12,11,11,10,10,9,8,8,7,7,
6,6,5,5,4,4,4,3,3,3,
2,2,2,1,1,1,1,1,0,0,
0,0,0,0,0,0,0,0,0,0,
0,0,0,1,1,1,1,1,2,2,
2,3,3,3,4,4,4,5,5,6,
6,7,7,8,8,9,10,10,11,11,
12,13,14,14,15,16,17,17,18,19,
20,21,22,22,23,24,25,26,27,28,
29,30,31,32,33,35,36,37,38,39,
40,41,43,44,45,46,47,49,50,51,
53,54,55,56,58,59,61,62,63,65,
66,67,69,70,72,73,75,76,78,79,
81,82,84,85,87,88,90,91,93,94,
96,97,99,100,102,104,105,107,108,110,
112,113,115,116,118,119,121,123,124,126};

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
    iter = iter%500;
    iter++;
    drive(sinLookUp[iter]);
    sin_step_trig = 0;
  }
  
  if(elapsedT > DISPLAY_RATE_uS) {
    Serial.println(String(timestamp) +  "," + String(sinLookUp[iter]) + "," + String(cntA) + "," + String(vel));
    elapsedT = 0;
  }
}
