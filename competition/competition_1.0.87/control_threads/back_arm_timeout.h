#pragma once

#include "vex.h"

using namespace vex;

vex::thread timeout_thread1;

int TIMEOUT1 = 0;
bool TIMED_OUT1 = false;

void reset_timeout1() {
  TIMED_OUT1 = false;
  TIMEOUT1 = 0;
}

int timeout1() {
  TIMED_OUT1 = false;
  this_thread::sleep_for(TIMEOUT1);
  TIMED_OUT1 = true;
  return 0;
}