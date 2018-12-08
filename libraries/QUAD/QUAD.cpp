#include "QUAD.h"

//Motor IDs
uint8_t L1HY = 0; //12/3/18
uint8_t L1HP = 1; //12/3/18
uint8_t L1K = 2; //12/3/18
uint8_t L1A = 3; //12/3/18
uint8_t L1D = 4; //12/3/18

uint8_t L2HY = 5; //12/3/18
uint8_t L2HP = 6; //12/3/18
uint8_t L2K = 7; //12/3/18
uint8_t L2A = 8; //12/3/18
uint8_t L2D = 9; //not ready

uint8_t L3HY = 10; //12/3/18
uint8_t L3HP = 11; //12/3/18
uint8_t L3K = 12; //12/3/18
uint8_t L3A = 13; //12/3/18
uint8_t L3D = 14; //12/3/18

uint8_t L4HY = 15; //12/3/18
uint8_t L4HP = 16; //12/3/18
uint8_t L4K = 17; //12/3/18
uint8_t L4A = 18; //12/3/18
uint8_t L4D = 19; //12/4/18

//motor offset
int16_t L1HY_offset = 500;
int16_t L1HP_offset = 500;
int16_t L1K_offset = 500;
int16_t L1A_offset = 500;
int16_t L1D_offset = 0;

int16_t L2HY_offset = 500;
int16_t L2HP_offset = 500;
int16_t L2K_offset = 500;
int16_t L2A_offset = 500;
int16_t L2D_offset = 0;

int16_t L3HY_offset = 500;
int16_t L3HP_offset = 500;
int16_t L3K_offset = 500;
int16_t L3A_offset = 500;
int16_t L3D_offset = 0;

int16_t L4HY_offset = 500;
int16_t L4HP_offset = 500;
int16_t L4K_offset = 500;
int16_t L4A_offset = 500;
int16_t L4D_offset = 0;

int16_t degrees2norm(float angle) {
  float ratio = angle/240;
  float resf = ratio*1000;
  return (int16_t)resf;
}

//names
uint16_t L1HY_ang;
uint16_t L1HP_ang;
uint16_t L1K_ang;
uint16_t L1A_ang;
uint16_t L1D_ang;

uint16_t L2HY_ang;
uint16_t L2HP_ang;
uint16_t L2K_ang;
uint16_t L2A_ang;
uint16_t L2D_ang;

uint16_t L3HY_ang;
uint16_t L3HP_ang;
uint16_t L3K_ang;
uint16_t L3A_ang;
uint16_t L3D_ang;

uint16_t L4HY_ang;
uint16_t L4HP_ang;
uint16_t L4K_ang;
uint16_t L4A_ang;
uint16_t L4D_ang;

uint16_t duration = 1500; //ms
uint8_t POS_CONTROL = 0x00;
uint8_t VEL_CONTROL = 0x01;
//int mspeed = 1000;

//define stand and sit parameters
float stand_hip_ang = 30;
float stand_knee_ang = -30;
//hold rest at 0 so the thing doesn't move

float sit_hip_ang = 80;
float sit_knee_ang = -80;

float sit_wheel_ang = 180;

void QUAD::default_pos(){
  LX16A::WRITE_MOVE_TIME(Serial,L1HY,(uint16_t)L1HY_offset,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L1HP,(uint16_t)L1HP_offset,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L1K,(uint16_t)L1K_offset,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L1A,(uint16_t)L1A_offset,duration);

  LX16A::WRITE_MOVE_TIME(Serial,L2HY,(uint16_t)L2HY_offset,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L2HP,(uint16_t)L2HP_offset,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L2K,(uint16_t)L2K_offset,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L2A,(uint16_t)L2A_offset,duration);

  LX16A::WRITE_MOVE_TIME(Serial,L3HY,(uint16_t)L3HY_offset,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L3HP,(uint16_t)L3HP_offset,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L3K,(uint16_t)L3K_offset,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L3A,(uint16_t)L3A_offset,duration);

  LX16A::WRITE_MOVE_TIME(Serial,L4HY,(uint16_t)L4HY_offset,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L4HP,(uint16_t)L4HP_offset,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L4K,(uint16_t)L4K_offset,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L4A,(uint16_t)L4A_offset,duration);
}

void QUAD::stand() {
  //default position
  //lock all joints or bring all joints to standing
  //thigh 30, knee -30
  //hold all wheels still
  LX16A::WRITE_MODE(Serial,L1D,VEL_CONTROL,0);
  LX16A::WRITE_MODE(Serial,L2D,VEL_CONTROL,0);
  LX16A::WRITE_MODE(Serial,L3D,VEL_CONTROL,0);
  LX16A::WRITE_MODE(Serial,L4D,VEL_CONTROL,0);

  L1A_ang = (uint16_t)(degrees2norm(45) + L1A_offset);
  L2A_ang = (uint16_t)(degrees2norm(-45) + L2A_offset);
  L3A_ang = (uint16_t)(degrees2norm(45) + L3A_offset);
  L4A_ang = (uint16_t)(degrees2norm(-45) + L4A_offset);

  LX16A::WRITE_MOVE_TIME(Serial,L1A,L1A_ang,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L2A,L2A_ang,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L3A,L3A_ang,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L4A,L4A_ang,duration);

  L1HP_ang = (uint16_t)(degrees2norm(-stand_hip_ang) + L1HP_offset);
  L1K_ang = (uint16_t)(degrees2norm(stand_hip_ang) + L1K_offset);
  L1D_ang = (uint16_t)(L1D_offset);
  LX16A::WRITE_MOVE_TIME(Serial,L1HP,L1HP_ang,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L1K,L1K_ang,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L1D,L1D_ang,duration);

  L2HP_ang = (uint16_t)(degrees2norm(-stand_hip_ang) + L2HP_offset);
  L2K_ang = (uint16_t)(degrees2norm(stand_hip_ang) + L2K_offset);
  L2D_ang = (uint16_t)(L2D_offset);
  LX16A::WRITE_MOVE_TIME(Serial,L2HP,L2HP_ang,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L2K,L2K_ang,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L2D,L2D_ang,duration);

  L3HP_ang = (uint16_t)(degrees2norm(-stand_hip_ang) + L3HP_offset);
  L3K_ang = (uint16_t)(degrees2norm(stand_hip_ang) + L3K_offset);
  L3D_ang = (uint16_t)(L3D_offset);
  LX16A::WRITE_MOVE_TIME(Serial,L3HP,L3HP_ang,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L3K,L3K_ang,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L3D,L3D_ang,duration);

  L4HP_ang = (uint16_t)(degrees2norm(-stand_hip_ang) + L4HP_offset);
  L4K_ang = (uint16_t)(degrees2norm(stand_hip_ang) + L4K_offset);
  L4D_ang = (uint16_t)(L4D_offset);
  LX16A::WRITE_MOVE_TIME(Serial,L4HP,L4HP_ang,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L4K,L4K_ang,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L4D,L4D_ang,duration);
  delay(duration);
}

void QUAD::stand2sit() {
  //move motors into the sitting position
  LX16A::WRITE_MODE(Serial,L1D,POS_CONTROL,0);
  LX16A::WRITE_MODE(Serial,L2D,POS_CONTROL,0);
  LX16A::WRITE_MODE(Serial,L3D,POS_CONTROL,0);
  LX16A::WRITE_MODE(Serial,L4D,POS_CONTROL,0);

  //Ankles to +-45 to face out
  LX16A::WRITE_MOVE_TIME(Serial,L1A,(uint16_t)L1A_offset,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L2A,(uint16_t)L2A_offset,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L3A,(uint16_t)L3A_offset,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L4A,(uint16_t)L4A_offset,duration);
  delay(duration);

  //Thigh at 90, knee -90, run wheels to allow sprawl
  L1HP_ang = (uint16_t)(degrees2norm(-sit_hip_ang) + L1HP_offset);
  L1K_ang = (uint16_t)(degrees2norm(sit_hip_ang) + L1K_offset);
  L1D_ang = (uint16_t)(degrees2norm(sit_wheel_ang) + L1D_offset);
  LX16A::WRITE_MOVE_TIME(Serial,L1HP,L1HP_ang,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L1K,L1K_ang,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L1D,L1D_ang,duration);

  L2HP_ang = (uint16_t)(degrees2norm(-sit_hip_ang) + L2HP_offset);
  L2K_ang = (uint16_t)(degrees2norm(sit_hip_ang) + L2K_offset);
  L2D_ang = (uint16_t)(degrees2norm(sit_wheel_ang) + L1D_offset);
  LX16A::WRITE_MOVE_TIME(Serial,L2HP,L2HP_ang,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L2K,L2K_ang,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L2D,L2D_ang,duration);

  L3HP_ang = (uint16_t)(degrees2norm(-sit_hip_ang) + L3HP_offset);
  L3K_ang = (uint16_t)(degrees2norm(sit_hip_ang) + L3K_offset);
  L3D_ang = (uint16_t)(degrees2norm(sit_wheel_ang) + L3D_offset);
  LX16A::WRITE_MOVE_TIME(Serial,L3HP,L3HP_ang,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L3K,L3K_ang,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L3D,L3D_ang,duration);

  L4HP_ang = (uint16_t)(degrees2norm(-sit_hip_ang) + L4HP_offset);
  L4K_ang = (uint16_t)(degrees2norm(sit_hip_ang) + L4K_offset);
  L4D_ang = (uint16_t)(degrees2norm(sit_wheel_ang) + L4D_offset);
  LX16A::WRITE_MOVE_TIME(Serial,L4HP,L4HP_ang,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L4K,L4K_ang,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L4D,L4D_ang,duration);
  delay(duration);
}

void QUAD::sit2stand(){
  //move motors from sitting to standing
  //ankles to +- 45 to face out
  //thigh at 30, knee -30, run wheels to allow sprawl
  //return wheels to 0
  L1HP_ang = (uint16_t)(degrees2norm(-stand_hip_ang) + L1HP_offset);
  L1K_ang = (uint16_t)(degrees2norm(stand_hip_ang) + L1K_offset);
  L1D_ang = (uint16_t)(L1D_offset);
  LX16A::WRITE_MOVE_TIME(Serial,L1HP,L1HP_ang,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L1K,L1K_ang,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L1D,L1D_ang,duration);

  L2HP_ang = (uint16_t)(degrees2norm(-stand_hip_ang) + L2HP_offset);
  L2K_ang = (uint16_t)(degrees2norm(stand_hip_ang) + L2K_offset);
  L2D_ang = (uint16_t)(L2D_offset);
  LX16A::WRITE_MOVE_TIME(Serial,L2HP,L2HP_ang,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L2K,L2K_ang,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L2D,L2D_ang,duration);

  L3HP_ang = (uint16_t)(degrees2norm(-stand_hip_ang) + L3HP_offset);
  L3K_ang = (uint16_t)(degrees2norm(stand_hip_ang) + L3K_offset);
  L3D_ang = (uint16_t)(L3D_offset);
  LX16A::WRITE_MOVE_TIME(Serial,L3HP,L3HP_ang,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L3K,L3K_ang,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L3D,L3D_ang,duration);

  L4HP_ang = (uint16_t)(degrees2norm(-stand_hip_ang) + L4HP_offset);
  L4K_ang = (uint16_t)(degrees2norm(stand_hip_ang) + L4K_offset);
  L4D_ang = (uint16_t)(L4D_offset);
  LX16A::WRITE_MOVE_TIME(Serial,L4HP,L4HP_ang,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L4K,L4K_ang,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L4D,L4D_ang,duration);

  delay(duration);
  L1A_ang = (uint16_t)(degrees2norm(45) + L1A_offset);
  L2A_ang = (uint16_t)(degrees2norm(-45) + L2A_offset);
  L3A_ang = (uint16_t)(degrees2norm(45) + L3A_offset);
  L4A_ang = (uint16_t)(degrees2norm(-45) + L4A_offset);
  LX16A::WRITE_MOVE_TIME(Serial,L1A,L1A_ang,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L2A,L2A_ang,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L3A,L3A_ang,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L4A,L4A_ang,duration);
  delay(duration);

}

int mspeed = 500;

void QUAD::rotate(uint16_t des_ang, uint16_t orientation){
  //counterclockwise is +1, clockwise is -1
  //turn ankles into rotate mode, run wheels depending on des_ang and orientation(left/right)
  //turn ankles +- 45 into rotate position (make circle)
  L1A_ang = (uint16_t)(degrees2norm(90) + L1A_offset);
  L2A_ang = (uint16_t)(degrees2norm(-90) + L2A_offset);
  L3A_ang = (uint16_t)(degrees2norm(90) + L3A_offset);
  L4A_ang = (uint16_t)(degrees2norm(-90) + L4A_offset);

  LX16A::WRITE_MOVE_TIME(Serial,L1A,L1A_ang,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L2A,L2A_ang,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L3A,L3A_ang,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L4A,L4A_ang,duration);
  delay(duration);
  //run wheels to desired angle in the desired direction/orientation
  //only can use velocity control here for a set time
  uint16_t theta_time = des_ang*100;
  int omspeed = orientation*mspeed;
  LX16A::WRITE_MODE(Serial,L1D,VEL_CONTROL,-omspeed);
  LX16A::WRITE_MODE(Serial,L2D,VEL_CONTROL,omspeed);
  LX16A::WRITE_MODE(Serial,L3D,VEL_CONTROL,-omspeed);
  LX16A::WRITE_MODE(Serial,L4D,VEL_CONTROL,omspeed);

  delay(theta_time);
  LX16A::WRITE_MODE(Serial,L1D,VEL_CONTROL,0);
  LX16A::WRITE_MODE(Serial,L2D,VEL_CONTROL,0);
  LX16A::WRITE_MODE(Serial,L3D,VEL_CONTROL,0);
  LX16A::WRITE_MODE(Serial,L4D,VEL_CONTROL,0);

  delay(1000);

  L1A_ang = (uint16_t)(degrees2norm(45) + L1A_offset);
  L2A_ang = (uint16_t)(degrees2norm(-45) + L2A_offset);
  L3A_ang = (uint16_t)(degrees2norm(45) + L3A_offset);
  L4A_ang = (uint16_t)(degrees2norm(-45) + L4A_offset);

  LX16A::WRITE_MOVE_TIME(Serial,L1A,L1A_ang,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L2A,L2A_ang,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L3A,L3A_ang,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L4A,L4A_ang,duration);
  delay(duration);

  //return ankles to 0
}

void QUAD::walk(uint16_t velocity,uint16_t wayward){
  //velocity goes from 1-10, wayward: forward = +1, backwards = -1
  //ensure wheels are all facing forward somehow
  //run wheels at velocity for desired duration

  L1A_ang = (uint16_t)(degrees2norm(45) + L1A_offset);
  L2A_ang = (uint16_t)(degrees2norm(-45) + L2A_offset);
  L3A_ang = (uint16_t)(degrees2norm(45) + L3A_offset);
  L4A_ang = (uint16_t)(degrees2norm(-45) + L4A_offset);

  LX16A::WRITE_MOVE_TIME(Serial,L1A,L1A_ang,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L2A,L2A_ang,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L3A,L3A_ang,duration);
  LX16A::WRITE_MOVE_TIME(Serial,L4A,L4A_ang,duration);

  delay(duration);
  //run motors... duration determined by UI
  int wmspeed = 1000*velocity/10;

  LX16A::WRITE_MODE(Serial,L1D,VEL_CONTROL,wayward*wmspeed);
  LX16A::WRITE_MODE(Serial,L2D,VEL_CONTROL,wayward*wmspeed);
  LX16A::WRITE_MODE(Serial,L3D,VEL_CONTROL,-wayward*wmspeed);
  LX16A::WRITE_MODE(Serial,L4D,VEL_CONTROL,-wayward*wmspeed);

}
