#pragma once

#include "vex.h"

using namespace vex;

vex::thread timeout_thread0;

int TIMEOUT0 = 0;
bool TIMED_OUT0 = false;

void reset_timeout0() {
  TIMED_OUT0 = false;
  TIMEOUT0 = 0;
}

int timeout0() {
  TIMED_OUT0 = false;
  this_thread::sleep_for(TIMEOUT0);
  TIMED_OUT0 = true;
  return 0;
}