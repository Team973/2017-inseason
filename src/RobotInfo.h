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
 * DIN pin-out
 */
constexpr int AIR_PRESSURE_DIN = 0;
constexpr int GEAR_INTAKE_BANNER_DIN = 18;

/**
 * Relays
 */
constexpr int COMPRESSOR_RELAY = 0;

/**
 * USB port-out (driver-station)
 */
constexpr int DRIVER_JOYSTICK_PORT = 0;
constexpr int OPERATOR_JOYSTICK_PORT = 1;

//solenoids
<<<<<<< HEAD
constexpr int GEAR_INTAKE_POS = 0;
constexpr int GEAR_INTAKE_GRIP_OPEN = 1;
constexpr int GEAR_INTAKE_GRIP_CLOSE = 2;
=======
constexpr int DRIVE_SHIFT_SOL = 0;
constexpr int GEAR_INTAKE_GRIP_A = 1;
constexpr int GEAR_INTAKE_GRIP_B = 2;
constexpr int GEAR_INTAKE_POS = 3;
>>>>>>> 8b27b27ceaa483262d3a3aeae95242800f03d284

constexpr int POWER_TAKEOFF_SOL_A = 3;
constexpr int POWER_TAKEOFF_SOL_B = 4;
//CANTalon
constexpr int DRIVE_LEFT_A_CAN = 16;
constexpr int DRIVE_LEFT_B_CAN = 15;
constexpr int DRIVE_RIGHT_A_CAN = 1;
constexpr int DRIVE_RIGHT_B_CAN = 2;

constexpr int SHOOTER_TURRET_CAN_ID = 2;
constexpr int HANGER_CAN_ID = 4;

constexpr int FLYWHEEL_PRIMARY_CAN_ID = 0;
constexpr int FLYWHEEL_REPLICA_CAN_ID = 1;

constexpr int LEFT_INDEXER_CAN_ID = 2;
constexpr int RIGHT_INDEXER_CAN_ID = 3;

constexpr int LEFT_AGITATOR_CAN_ID = 4;
constexpr int RIGHT_AGITATOR_CAN_ID = 5;

constexpr int BALL_INTAKE_CAN_ID = 6;
constexpr int BALL_INTAKE_B_CAN_ID = 7;
constexpr int BALL_CONVEYOR_CAN_ID = 8;

constexpr int FLYWHEEL_CONTROL_PERIOD_MS = 1;
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

constexpr double SHOOTER_CLICKS_PER_REVOLUTION = 360.0;
}

#endif /* SRC_ROBOTINFO_H_ */
