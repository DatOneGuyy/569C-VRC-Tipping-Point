/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\compu                                            */
/*    Created:      Sat Nov 20 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// left_drive           motor_group   1, 8            
// right_drive          motor_group   11, 16          
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>

using namespace vex;

competition Competition;

double PI = 3.141592653589793;
double TWO_PI = PI * 2;

double ta2(double a) {return a > 0 ? (a > PI ? a - TWO_PI : a) : (a < -PI ? TWO_PI + a : a);}
int sgn(double x) {return x == 0 ? 0 : std::abs(x) / x;}
double constrain(double value, double min, double max) {return value < min ? min : (value > max ? max : value);}
double dtoi(double degrees) {return degrees / 360 * PI * 3.25;}

double leftspeed = 0;
double rightspeed = 0;

double leftdrive = 0;
double rightdrive = 0;

double x = 0;
double y = 0;
double heading = PI / 2;

double trackwidth = 8;

double kp = 0.9;
double normalkp = 0.9;
double boostkp = 2.0;
double boostkpthreshold = 0.5;

double originalspeed = 0;
double originalx = 0;
double originaly;

double netanglechange = 0;
double leftchange = 0;
double rightchange = 0;
double leftprev = 0;
double rightprev = 0;

double chorddistance = 0;

void odometry() {/*
  leftspeed = constrain(leftspeed, -12.0, 12.0);
  rightspeed = constrain(rightspeed, -12.0, 12.0);
  left_drive.spin(fwd, leftspeed, volt);
  right_drive.spin(fwd, rightspeed, volt);*/
  
  leftdrive = dtoi(left_drive.position(degrees));
  rightdrive = dtoi(right_drive.position(degrees));
  leftchange = leftdrive - leftprev;
  rightchange = rightdrive - rightprev;
  leftprev = leftdrive;
  rightprev = rightdrive;

  netanglechange = (leftchange - rightchange) / trackwidth;
  chorddistance = 2 * std::sin(netanglechange / 2 * (rightspeed / netanglechange + trackwidth / 2));

  x += chorddistance * cos(netanglechange + heading);
  y += chorddistance * sin(netanglechange + heading);
  heading += netanglechange;
  heading = ta2(std::fmod(heading, TWO_PI));
  
  Brain.Screen.print("X: %d inches, Y: %d inches, Heading: %d degrees", x, y, heading * 180 / PI);
}

void pre_auton() {

}

void autonomous() {
  while (true) {

  }
}

void drivercontrol() {
  while (true) {
    odometry();
  }
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  pre_auton();

  Competition.autonomous(autonomous);
  Competition.drivercontrol(drivercontrol);

  Competition.bStopAllTasksBetweenModes = true;
}
