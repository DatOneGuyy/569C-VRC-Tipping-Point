#include "vex.h"

using namespace vex;

int max_lines = 10;
int lines = 0;



void pd(double x) {
  if (lines > 10) {
    Brain.Screen.clearScreen();
  }
  Brain.Screen.print(x);  
  Brain.Screen.newLine();
  lines++;
}

