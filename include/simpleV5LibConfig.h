#include "vex.h"

using namespace vex;

const int PORT_LEFTFRONT = PORT1; // left front
const int PORT_LEFTMIDDLE = PORT2; // left middle, or left back, depends on your drive config
const int PORT_LEFTBACK = PORT3; // left back, or left high, depends on your drive config
const int PORT_RIGHTFRONT = PORT4;
const int PORT_RIGHTMIDDLE = PORT5;
const int PORT_RIGHTBACK = PORT6;
const int PORT_INERTIAL = PORT7;

const gearSetting LF_GEAR_RATIO = ratio18_1; // half motor
const gearSetting LM_GEAR_RATIO = ratio6_1; // full motor
const gearSetting LB_GEAR_RATIO = ratio6_1; // full motor
const gearSetting RF_GEAR_RATIO = ratio18_1; // half motor
const gearSetting RM_GEAR_RATIO = ratio6_1; // full motor
const gearSetting RB_GEAR_RATIO = ratio6_1; // full motor

 // true = reversed, false = not reversed
const bool LF_DIRECTION = true;
const bool LM_DIRECTION = true;
const bool LB_DIRECTION = false;
const bool RF_DIRECTION = false;
const bool RM_DIRECTION = false;
const bool RB_DIRECTION = true;

// Tune PID turn constants
const double TURN_KP = 3.2;
const double TURN_KI = 0.2;
const double TURN_KD = 36.7;

// Turn integral range
const double TURN_INTEGRAL_RANGE = 10;

// Tune PID turn constnats
const double FORWARD_KP = 12;
const double FORWARD_KI = 0.05;
const double FORWARD_KD = 0.78;

// Turn integral range
const double FORWARD_INTEGRAL_RANGE = 1.5;

#define WHEEL_DIAMETER_INCH 3.25
#define WHEEL_CIRCUMFERENCE_INCH WHEEL_DIAMETER_INCH * M_PI
#define MOTOR_TO_WHEEL_GEAR_RATIO 2.0/3.0