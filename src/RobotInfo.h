/*
 * RobotInfo.h
 *
 *  Created on: Oct 9, 2015
 *      Author: Andrew
 */

#ifndef ROBOTINFO_H_
#define ROBOTINFO_H_

#include "lib/util/Util.h"

namespace frc973 {

//#define PROTO_BOT_PINOUT
//#define PRACTICE_BOT_PINOUT

/**
 * PWM pin-out
 */
#ifdef PRACTICE_BOT_PINOUT
constexpr int DRIVE_RIGHT_PWM = 0;
constexpr int DRIVE_LEFT_PWM = 1;
#else
constexpr int DRIVE_RIGHT_PWM = 1;
constexpr int DRIVE_LEFT_PWM = 0;
#endif

constexpr int FRONT_SHOOTER_PWM = 9;
constexpr int BACK_SHOOTER_PWM = 3;

//constexpr int UNUSED_PWM = 4;

constexpr int LINEAR_EXTENSION_PWM = 5;

constexpr int SHOOTER_CONVEYER_MOTOR_PWM = 6;
constexpr int BALL_INTAKE_MOTOR_PWM = 7;

constexpr int ARM_MOTOR_PWM = 8;
/**
 * DIN pin-out
 */
constexpr int AIR_PRESSURE_DIN = 0;

constexpr int PIXY_CAM_DIGITAL_PORT = 9;

constexpr int LEFT_HOOK_HALL_DIN = 3;
constexpr int RIGHT_HOOK_HALL_DIN = 4;

constexpr int RIGHT_DRIVE_ENCODER_A_DIN = 12;
constexpr int RIGHT_DRIVE_ENCODER_B_DIN = 13;
constexpr int LEFT_DRIVE_ENCODER_A_DIN = 24;
constexpr int LEFT_DRIVE_ENCODER_B_DIN = 25;

#ifdef PROTO_BOT_PINOUT
constexpr int FLYWHEEL_FRONT_BANNERSENSOR_DIN = 6;
constexpr int FLYWHEEL_BACK_BANNERSENSOR_DIN = 7;

constexpr int ARM_ENCODER_A_DIN = 20;
constexpr int ARM_ENCODER_B_DIN = 21;

constexpr int COLLIN_GYRO_A_DIN = 0;
constexpr int COLLIN_GYRO_B_DIN = 1;
#else
constexpr int FLYWHEEL_FRONT_BANNERSENSOR_DIN = 7;
constexpr int FLYWHEEL_BACK_BANNERSENSOR_DIN = 8;

constexpr int ARM_ENCODER_A_DIN = 22;
constexpr int ARM_ENCODER_B_DIN = 23;
#endif

/**
 * Analog In
 */
constexpr int PIXY_CAM_ANALOG_PORT = 0;

/**
 * Relay pin-out
 */

constexpr int COMPRESSOR_RELAY = 0;

/**
 * Solenoid channels
 */
constexpr int SHOOTER_ANGLE_UPPER_SOL = 0;
constexpr int SHOOTER_ANGLE_LOWER_SOL = 1;

#ifdef PRACTICE_BOT_PINOUT
constexpr int DRIVE_BREAK_SOL_A = 3;
constexpr int DRIVE_BREAK_SOL_B = 5;
constexpr int DRIVE_SHIFT_SOL = 2;
#else
constexpr int DRIVE_SHIFT_SOL = 3;
#endif

constexpr int INTAKE_EXTENSION_SOL = 4;

constexpr int POWER_TAKEOFF_SOL_A = 2;
constexpr int POWER_TAKEOFF_SOL_B = 5;

constexpr int READY_LIGHT_SOL = 7;
constexpr int RUNNING_LIGHT_SOL = 6;
/**
 * Power Distrobution Channels
 * (only the ones we care about)
 */
#ifdef PRACTICE_BOT_PINOUT
constexpr int BACK_FLYWHEEL_PDB = 2;
constexpr int ARM_PDB = 6;
#else
constexpr int BACK_FLYWHEEL_PDB = 12;
constexpr int ARM_PDB = 5;
#endif

constexpr double BACK_FLYWHEEL_STALL_CURRENT = 50.0;
/**
 * USB port-out (driver-station)
 */
constexpr int DRIVER_JOYSTICK_PORT = 0;
constexpr int OPERATOR_JOYSTICK_PORT = 1;

/**
 * Distance (in inches) of the drive per click of the encoder
 */
constexpr double DRIVE_WHEEL_DIAMETER = 4.0;
constexpr double DRIVE_CLICKS_PER_REVOLUTION = 360.0;
constexpr double DRIVE_GEAR_RATIO = 1.0;
constexpr double DRIVE_DIST_PER_REVOLUTION = DRIVE_WHEEL_DIAMETER *
		Constants::PI;
constexpr double DRIVE_DIST_PER_CLICK = DRIVE_DIST_PER_REVOLUTION *
		DRIVE_CLICKS_PER_REVOLUTION;

/**
 * Distance (in degrees) of the arm per click of the encoder (after
 * compensating for sampling)
 */
constexpr double ARM_DIST_PER_CLICK = 1.0 * 9.0 / 30.0;

}

#endif /* SRC_ROBOTINFO_H_ */
