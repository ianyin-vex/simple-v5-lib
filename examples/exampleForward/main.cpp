/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       ianyin                                                    */
/*    Created:      6/28/2026, 16:09:01 PM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "simpleV5lib.h"

using namespace vex;

void pre_auton(void) {
}

void autonomous(void) {
}

void usercontrol(void) {
}

int main() {
    PID_forward(23.622, 0.3, 0.2);
    // The first parameter is target distance, in inches.
    // The second parameter is the error tolerance, in inches. This is the value in which if the error (distance to the target), is within, the movement will end
    // The second parameter is the speed tolerance, in inch/s. again, saim logic with error tolerance.
    // The fourth parameter is the speed percentage, which controls the max speed the robot can go as a percentage. (max is 1, min is 0)
}