#ifndef QUAD_H
#define QUAD_H

#include "LX16A.h"

namespace {
  void default_pos();
  void stand();
  void stand2sit();
  void sit2stand();
  void rotate(uint16_t des_ang, uint16_t orientation ) //+1 for counterclockwise, -1 for clockwise
  void walk(uint16_t velocity, uint16_t wayward); //+1 for forward, -1 for reverse
}
