#pragma once

#include "vex.h"
#include <cmath>

using namespace vex;

//average
double average(int a, int b) { 
  return 0.5 * (a + b);
}

double daverage(int a, int b) {
  return 0.5 * (a + b);
}

double angleConvert(double angle) {
  return angle > 180 ? angle - 360 : angle;
}

double normalize(double x) {
  return x == 0 ? 0 : std::abs(x) / x;
}

//curve of degree 2n, higher -> faster with diminishing returns, lower -> slower but smooth deceleration
//domain: [0, 100], proportion of path completed
//range: [0, 100], multiplier for speed
double curve(double n, double x) { 
  if (x < 50) {
    return 15 + 5 * x;
  } else {
    return std::pow(std::pow(100, 2 * n) - std::pow(x, 2 * n), 1 / (2 * n));
  }
}

double linear_accel(double x) {
  return 40 + 4 * x > 100 ? 100 : 40 + 4 * x;
}

double linear(double x, double x1, double y1, double x2, double y2) {
  return (y2 - y1) / (x2 - x1) * (x - x1) + y1;
}

double inverse_linear(double x, double x1, double y1, double x2, double y2) {
  return (x2 - x1) / (y2 - y1) * (x - y1) + x1;
}

//motor speed to power map (need to record values sometime)
int map(double input) {
  double value = 0;
  double x = input * 1.3;

  if (x < 12) {
    value = inverse_linear(x, 0, 0, 15, 12);
  } else if (x < 19) {
    value = inverse_linear(x, 15, 12, 20, 19);
  } else if (x < 26) {
    value = inverse_linear(x, 20, 19, 25, 26);
  } else if (x < 32) {
    value = inverse_linear(x, 25, 26, 30, 32);
  } else if (x < 39) {
    value = inverse_linear(x, 30, 32, 35, 39);
  } else if (x < 47) {
    value = inverse_linear(x, 35, 39, 40, 47);
  } else if (x < 53) {
    value = inverse_linear(x, 40, 47, 45, 53);
  } else if (x < 66) {
    value = inverse_linear(x, 45, 53, 50, 66);
  } else if (x < 69) {
    value = inverse_linear(x, 50, 66, 55, 69);
  } else if (x < 75) {
    value = inverse_linear(x, 55, 69, 60, 75);
  } else if (x < 82) {
    value = inverse_linear(x, 60, 75, 65, 82);
  } else if (x < 88) {
    value = inverse_linear(x, 65, 82, 70, 88);
  } else if (x < 95) {
    value = inverse_linear(x, 70, 88, 75, 95);
  } else if (x < 102) {
    value = inverse_linear(x, 75, 95, 80, 102);
  } else if (x < 108) {
    value = inverse_linear(x, 80, 102, 85, 108);
  } else if (x < 114) {
    value = inverse_linear(x, 85, 108, 90, 114);
  } else if (x < 121) {
    value = inverse_linear(x, 90, 114, 95, 121);
  } else {
    value = inverse_linear(x, 95, 121, 100, 130);
  } 

  return (int)value;
}