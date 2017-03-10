/*
 * RobotInfo.h
 *
 *  Created on: Oct 9, 2015
 *      Author: Andrew
 */

#pragma once

#include "lib/util/Util.h"

namespace frc973 {

//#define PROTO_BOT_PINOUT
//#define PRACTICE_BOT_PINOUT

/**
 * DIN pin-out
 */
constexpr int AIR_PRESSURE_DIN = 0;
constexpr int GEAR_INTAKE_BANNER_DIN = 18;

constexpr int PUSH_SENSOR_TOP_LEFT = 1;
constexpr int PUSH_SENSOR_TOP_RIGHT = 2;
constexpr int PUSH_SENSOR_BOTTOM = 3;

constexpr int BOILER_PIXY_CAM_X_DIGITAL = 4;
constexpr int BOILER_PIXY_CAM_Y_DIGITAL = 5;
constexpr int GEAR_PIXY_CAM_DIGITAL = 6;

constexpr int BOILER_PIXY_CAM_X_ANALOG = 0;
constexpr int BOILER_PIXY_CAM_Y_ANALOG = 1;
constexpr int GEAR_PIXY_CAM_ANALOG = 2;
/**
 * Relays
 */
constexpr int COMPRESSOR_RELAY = 0;

/**
 * USB port-out (driver-station)
 */
constexpr int DRIVER_JOYSTICK_PORT = 0;
constexpr int OPERATOR_JOYSTICK_PORT = 1;
constexpr int TUNING_JOYSTICK_PORT = 2;

//solenoids
constexpr int GEAR_INTAKE_POS = 0;
constexpr int GEAR_INTAKE_GRIP_OPEN = 1;
constexpr int GEAR_INTAKE_GRIP_CLOSE = 2;

constexpr int BOILER_PIXY_LIGHT_SOL = 6;

constexpr int FLASH_LIGHT_SOL = 4;
//CANTalon
constexpr int DRIVE_LEFT_A_CAN = 16;
constexpr int DRIVE_LEFT_B_CAN = 15;
constexpr int DRIVE_RIGHT_A_CAN = 1;
constexpr int DRIVE_RIGHT_B_CAN = 2;

constexpr int HANGER_CAN_ID = 4;
constexpr int HANGER_CAN_ID_B = 13;

constexpr int FLYWHEEL_PRIMARY_CAN_ID = 14;
constexpr int FLYWHEEL_REPLICA_CAN_ID = 3;

constexpr int LEFT_INDEXER_CAN_ID = 12;
constexpr int RIGHT_INDEXER_CAN_ID = 11;

constexpr int LEFT_AGITATOR_CAN_ID = 10;
constexpr int RIGHT_AGITATOR_CAN_ID = 6;

constexpr int BALL_INTAKE_CAN_ID = 7;
constexpr int BALL_CONVEYOR_CAN_ID = 5;

constexpr int SPARE_TALON_A = 61;
constexpr int SPARE_TALON_B = 62;
//default rate is 10ms
constexpr int FLYWHEEL_CONTROL_PERIOD_MS = 5;
/**
 * Distance (in inches) of the drive per click of the encoder
 */
constexpr double DRIVE_WHEEL_DIAMETER = 3.25;

constexpr double DRIVE_DIST_PER_REVOLUTION =
    DRIVE_WHEEL_DIAMETER * Constants::PI;
constexpr double DRIVE_WIDTH = 23.0;
//inches/sec from revolutions/minute
constexpr double DRIVE_IPS_FROM_RPM = 
    DRIVE_DIST_PER_REVOLUTION / 60.0;
}
