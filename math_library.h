#include "vex.h"
#include <cmath>

using namespace vex;

//average
double average(int a, int b) { 
  return 0.5 * (a + b);
}

//returns larger value
double max(double a, double b) { 
  return a >= b ? a : b;
}

//curve of degree 2n, higher -> faster with diminishing returns, lower -> slower but smooth deceleration
//domain: [0, 100], proportion of path completed
//range: [0, 100], multiplier for speed
double curve(double n, double x) { 
  return pow(pow(100, 2 * n) - pow(x, 2 * n), 1 / (2 * n));
}

//motor speed to power map (need to record values sometime)
int map(double x) {
  /*
  int adjusted [100];
  return adjusted[(int) round(x)];
  */
  return (int) x;
}
