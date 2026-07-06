#include "simpleV5lib.h"
#include <cstdio>
#include <cstdlib>
#include <iostream>

brain Brain;
controller Controller;
motor leftFront(PORT_LEFTFRONT, LF_GEAR_RATIO, LF_DIRECTION);
motor leftBack(PORT_RIGHTMIDDLE, LM_GEAR_RATIO, LM_DIRECTION);
motor leftHigh(PORT_LEFTBACK, LB_GEAR_RATIO, LB_DIRECTION);
motor rightFront(PORT_RIGHTFRONT, RF_GEAR_RATIO, RF_DIRECTION);
motor rightBack(PORT_RIGHTMIDDLE, RM_GEAR_RATIO, RM_DIRECTION);
motor rightHigh(PORT_RIGHTBACK, RB_GEAR_RATIO, RB_DIRECTION);

inertial Inertial(PORT_INERTIAL);

double getInertial() {
    return Inertial.rotation(rotationUnits::deg);
}

double getGyroRate() {
    return Inertial.gyroRate(zaxis, dps) / 100; // degrees/10 ms
}

double getPosition() {
    return rightFront.position(rotationUnits::rev) * WHEEL_CIRCUMFERENCE_INCH * MOTOR_TO_WHEEL_GEAR_RATIO;
}

double getMotorRate() {
    return leftFront.velocity(velocityUnits::dps) / 360 * WHEEL_CIRCUMFERENCE_INCH * MOTOR_TO_WHEEL_GEAR_RATIO; // in/s
}

void stopDriving() {
    leftFront.stop(brakeType::brake);
    leftBack.stop(brakeType::brake);
    leftHigh.stop(brakeType::brake);
    rightFront.stop(brakeType::brake);
    rightBack.stop(brakeType::brake);
    rightHigh.stop(brakeType::brake);
}

void move(double left_speed, double right_speed) {
    left_speed *= 120;
    right_speed *= 120;
    leftFront.spin(directionType::fwd, left_speed, voltageUnits::mV);
    leftBack.spin(directionType::fwd, left_speed, voltageUnits::mV);
    leftHigh.spin(directionType::fwd, left_speed, voltageUnits::mV);
    rightFront.spin(directionType::fwd, right_speed, voltageUnits::mV);
    rightBack.spin(directionType::fwd, right_speed, voltageUnits::mV);
    rightHigh.spin(directionType::fwd, right_speed, voltageUnits::mV);
}

double cap(double input, uint32_t max_value) {
    if (fabs(input) > max_value) {
        return (input >= 0) ? max_value : max_value * -1.0;
    }
    return input;
}

double getSign(double input) {
    if (input >= 0) {
        return 1;
    } else {
        return -1;
    }
}

void PID_turn(double target, double error_tolerance, double speed_tolerance) {
    long delay = 10;
    double kp = TURN_KP;
    double ki = TURN_KI;
    double kd = TURN_KD;
    double porportional_correction = 0;
    double integral_correction = 0;
    double derivative_correction = 0;
    double current_heading = getInertial();
    double target_heading = target;
    double current_error = target_heading - current_heading;
    double past_error = current_error;
    double error_sum = 0;
    double total_correction = 0;
    double integral_range = TURN_INTEGRAL_RANGE;

    while(fabs(current_error) > error_tolerance || fabs(getGyroRate()) > speed_tolerance) {
        current_heading = getInertial();
        current_error = target_heading - current_heading;

        if (fabs(current_error) < integral_range) {
            error_sum = error_sum + current_error;
        } else {
            error_sum = 0;
        }
        if ((current_error * past_error) < 0) {
            error_sum = 0;
        }
        porportional_correction = current_error * kp;
        integral_correction = error_sum * ki;
        derivative_correction = getGyroRate() * kd;
        total_correction = porportional_correction + integral_correction + derivative_correction;

        if (fabs(total_correction) < 10 && fabs(getGyroRate()) < speed_tolerance) {
            total_correction = getSign(total_correction) * 10;
        }
        
        move(total_correction, total_correction * -1);
        past_error = current_error;
        vexDelay(delay);
    }
    move(0, 0);
}

void PID_forward(double target, double error_tolerance, double speed_tolerance) {
    double startTime = Brain.timer(timeUnits::sec);
    long delay = 10;
    double kp = FORWARD_KP;
    double ki = FORWARD_KI;
    double kd = FORWARD_KD;

    double porportional_correction = 0;
    double integral_correction = 0;
    double derivative_correction = 0;
    rightFront.setPosition(0, rotationUnits::deg);
    double current_position = getPosition();
    double target_distance = target;
    double current_error = target_distance - current_position;
    double past_error = current_error;
    double error_sum = 0;
    double total_correction = 0;
    double integral_range = FORWARD_INTEGRAL_RANGE;
    double motorRate = getMotorRate();
    double current_time = 0;

    while (fabs(current_error) > error_tolerance || fabs(motorRate) > speed_tolerance) {
        current_time = Brain.timer(timeUnits::sec) - startTime;
        current_position = getPosition();
        current_error = target - current_position;

        if (fabs(current_error) < integral_range) {
            error_sum = error_sum + current_error;
        } else {
            error_sum = 0;
        }
        if ((current_error * past_error) < 0) {
            error_sum = 0;
        }
        porportional_correction = current_error * kp;
        integral_correction = error_sum * ki;
        derivative_correction = motorRate * kd;
        total_correction = cap(porportional_correction + integral_correction + derivative_correction, 100);
        if (current_time < 0.3 && total_correction > 60) {
            total_correction = getSign(target) * (30 + (current_time * 233));
        }

        move(total_correction, total_correction);
        past_error = current_error;
        vexDelay(delay);
        motorRate = getMotorRate();
    }
    move(0, 0);
}