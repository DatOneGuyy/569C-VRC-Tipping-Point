#pragma once

#include "vex.h"

using namespace vex;

vex::thread timeout_thread2;

int TIMEOUT2 = 0;
bool TIMED_OUT2 = false;

void reset_timeout2() {
  TIMED_OUT2 = false;
  TIMEOUT2 = 0;
}

int timeout2() {
  TIMED_OUT2 = false;
  this_thread::sleep_for(TIMEOUT2);
  TIMED_OUT2 = true;
  return 0;
}