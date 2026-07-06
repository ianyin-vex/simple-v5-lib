#ifndef SIMPLEV5LIB_H
#define SIMPLEV5LIB_H
#include "simpleV5LibConfig.h"

// Declare global variables

using namespace vex;

extern brain Brain;
extern controller Controller;
extern motor leftFront;
extern motor leftMiddle;
extern motor leftBack;
extern motor rightFront;
extern motor rightMiddle;
extern motor rightBack;

extern inertial Inertial;

void move(double left_speed, double right_speed);
void PID_turn(double target, double error_tolerance, double speed_tolerance);
void PID_forward(double target, double error_tolerance, double speed_tolerance);

double getInertial();

double getGyroRate();

double getPosition();

double getMotorRate();
#endif