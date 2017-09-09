#include <stdio.h>

#include "RobotInfo.h"

#include "subsystems/Drive.h"
#include "WPILib.h"
#include "lib/util/Util.h"

#include "lib/filters/RampedOutput.h"

#include "lib/logging/LogSpreadsheet.h"
#include "lib/WrapDash.h"

#include "controllers/ArcadeDriveController.h"
#include "controllers/OpenloopArcadeDriveController.h"
#include "controllers/AssistedArcadeDrive.h"
#include "controllers/PIDDrive.h"
#include "controllers/BoilerPixyVisionDriveController.h"
#include "controllers/GearPixyVisionDriveController.h"
#include "controllers/TrapDriveController.h"
#include "controllers/StraightDriveController.h"
#include "controllers/SplineDriveController.h"
#include "lib/SPIGyro.h"

namespace frc973 {

Drive::Drive(TaskMgr *scheduler, CANTalon *left, CANTalon *right,
            CANTalon *spareTalon,
            LogSpreadsheet *logger, BoilerPixy *boilerPixy, PixyThread *gearPixy,
            ADXRS450_Gyro *gyro
            )
         : DriveBase(scheduler, this, this, nullptr)
         , m_austinGyro(gyro)
         , m_angle(0.0)
         , m_angleRate(0.0)
         , m_leftCommand(0.0)
         , m_rightCommand(0.0)
         , m_leftMotor(left)
         , m_rightMotor(right)
         , m_controlMode(CANSpeedController::ControlMode::kPercentVbus)
         , m_spreadsheet(logger)
         , m_boilerPixyDriveController(
                 new BoilerPixyVisionDriveController(boilerPixy))
         , m_gearPixyDriveController(
                 new GearPixyVisionDriveController(gearPixy))
         , m_angleLog(new LogCell("Angle"))
         , m_angularRateLog(new LogCell("Angular Rate"))
         , m_leftDistLog(new LogCell("Left Encoder Distance"))
         , m_leftDistRateLog(new LogCell("Left Encoder Rate"))
         , m_rightDistLog(new LogCell("Right Encoder Distance"))
         , m_rightDistRateLog(new LogCell("Right Encoder Rate"))
         , m_leftCommandLog(new LogCell("Left motor signal (pow or vel)"))
         , m_rightCommandLog(new LogCell("Right motor signal (pow or vel)"))
         , m_leftVoltageLog(new LogCell("Left motor voltage"))
         , m_rightVoltageLog(new LogCell("Right motor voltage"))
         , m_currentLog(new LogCell("Drive current"))
{
    fprintf(stderr, "Initializing Drive Subsystem %p\n", this);
    fprintf(stderr, "Survived fprintf yes its up to date\n");

    m_arcadeDriveController = new ArcadeDriveController();
    m_openloopArcadeDriveController = new OpenloopArcadeDriveController();
    m_assistedArcadeDriveController = new AssistedArcadeDriveController();
    m_pidDriveController = new PIDDriveController();
    m_trapDriveController = new TrapDriveController(this, logger);
    m_straightDriveController = new StraightDriveController();
    m_splineDriveController = new SplineDriveController(this, logger);
    this->SetDriveController(m_arcadeDriveController);
    this->SetDriveControlMode(m_controlMode);

    bool loggingEnabled = true;
    if (loggingEnabled) {
        m_spreadsheet->RegisterCell(m_angleLog);
        m_spreadsheet->RegisterCell(m_angularRateLog);
        m_spreadsheet->RegisterCell(m_leftDistLog);
        m_spreadsheet->RegisterCell(m_leftDistRateLog);
        m_spreadsheet->RegisterCell(m_leftCommandLog);
        m_spreadsheet->RegisterCell(m_rightDistLog);
        m_spreadsheet->RegisterCell(m_rightDistRateLog);
        m_spreadsheet->RegisterCell(m_rightCommandLog);
        m_spreadsheet->RegisterCell(m_leftVoltageLog);
        m_spreadsheet->RegisterCell(m_rightVoltageLog);
        m_spreadsheet->RegisterCell(m_currentLog);
    }
    fprintf(stderr, "Enabled spreadsheets\n");

    scheduler->RegisterTask("Drive", this, TASK_PERIODIC);
    fprintf(stderr, "Scheduled task\n");
}

/**
 *  Zeroes gyro, left drive pos, and right drive pos
 */
void Drive::Zero() {
    if (m_austinGyro) {
        m_gyroZero = m_austinGyro->GetAngle();
    }
    if (m_leftMotor) {
        m_leftPosZero = m_leftMotor->GetPosition() * DRIVE_DIST_PER_REVOLUTION;
    }
    if (m_rightMotor) {
        m_rightPosZero = -m_rightMotor->GetPosition() * DRIVE_DIST_PER_REVOLUTION;
    }
}

/**
 * Sets Drive controller to ArcadeDrive
 *
 * @param throttle  Left joystick y-axis value
 * @param turn      Right joystick x-axis value
 */
void Drive::ArcadeDrive(double throttle, double turn) {
    this->SetDriveController(m_arcadeDriveController);
    m_arcadeDriveController->SetJoysticks(throttle, turn);
}

/**
 * Sets Drive controller to OpenLoopArcadeDrive
 *
 * @param throttle  Left joystick y-axis value
 * @param turn      Right joystick x-axis value
 */
void Drive::OpenloopArcadeDrive(double throttle, double turn) {
    this->SetDriveController(m_openloopArcadeDriveController);
    m_openloopArcadeDriveController->SetJoysticks(throttle, turn);
}

/**
 * Sets Drive controller to AssistedArcadeDrive
 *
 * @param throttle  Left joystick y-axis value
 * @param turn      Right joystick x-axis value
 */
void Drive::AssistedArcadeDrive(double throttle, double turn){
  this->SetDriveController(m_assistedArcadeDriveController);
  m_assistedArcadeDriveController->SetJoysticks(throttle, turn);
}

/**
 * Sets Drive controller to BoilerPixyVisionDriveController
 */
void Drive::SetBoilerPixyTargeting(){
  this->SetDriveController(m_boilerPixyDriveController);
}

/**
 * Sets Drive controller to GearPixyVisionDriveController
 */
void Drive::SetGearPixyTargeting(){
  this->SetDriveController(m_gearPixyDriveController);
}

/**
 * Sets Drive mode to PIDDrive
 *
 * @param dist        Desired drive distance
 * @param turn        Desired turn angle
 * @param relativity  Calculates relativity of drive distance and current position
 * @param powerCap    Percentage of drive power; 1.0 = full power
 *
 * @return            the PID Drive contoller
 */
PIDDriveController *Drive::PIDDrive(double dist, double turn,
        RelativeTo relativity, double powerCap)
{
    this->SetDriveController(m_pidDriveController);
    m_pidDriveController->SetCap(powerCap);
    m_pidDriveController->SetTarget(dist, turn,
            relativity, this);
    m_pidDriveController->EnableDist();
    return m_pidDriveController;
}

/**
 * Sets Drive mode to PIDTurn
 *
 * @param turn        Desired turn angle
 * @param relativity  Calculates relativity of drive distance and current position
 * @param powerCap    Percentage of drive power; 1.0 = full power
 *
 * @return            the PID Drive contoller
 */
PIDDriveController *Drive::PIDTurn(double turn, RelativeTo relativity,
        double powerCap)
{
    this->SetDriveController(m_pidDriveController);
    m_pidDriveController->SetCap(powerCap);
    m_pidDriveController->SetTarget(0.0, turn,
            relativity, this);
    m_pidDriveController->DisableDist();
    return m_pidDriveController;
}

/**
 * Returns Left Drive Distance thorugh encoder translation
 *
 * @return  Left Drive Distance reported in inches
 */
double Drive::GetLeftDist() const {
    return m_leftMotor->GetPosition() * DRIVE_DIST_PER_REVOLUTION -
        m_leftPosZero;
}

/**
 * Returns Right Drive Distance thorugh encoder translation
 *
 * @return  Right Drive Distance reported in inches
 */
double Drive::GetRightDist() const {
    return -m_rightMotor->GetPosition() * DRIVE_DIST_PER_REVOLUTION -
        m_rightPosZero;
}

/**
 * Returns Left Drive Rate or speed thorugh encoder translation
 *
 * @return  Left Drive Rate or Speed reported in inches Reported in inches per second; As per manual 17.2.1, GetSpeed reports RPM
 */
double Drive::GetLeftRate() const {
    return m_leftMotor->GetSpeed() * DRIVE_IPS_FROM_RPM;
}

/**
 * Returns Right Drive Rate or speed thorugh encoder translation
 *
 * @return  Right Drive Rate or Speed reported in inches Reported in inches per second; As per manual 17.2.1, GetSpeed reports RPM
 */
double Drive::GetRightRate() const {
    return -m_rightMotor->GetSpeed() * DRIVE_IPS_FROM_RPM;
}

/**
 * Returns Average Drive Distance thorugh encoder translation
 *
 * @return  Average Drive Distance reported in inches
 */
double Drive::GetDist() const {
    return (GetLeftDist() + GetRightDist()) / 2.0;
}

/**
 * Returns Average Drive Rate or speed thorugh encoder translation
 *
 * @return  Average Drive Rate or Speed reported in inches Reported in inches per second; As per manual 17.2.1, GetSpeed reports RPM
 */
double Drive::GetRate() const {
    return (GetLeftRate() + GetRightRate()) / 2.0;
}

/**
 * Returns Average Drive Current thorugh Talon SRX Output
 *
 * @return  Avergage current reported in amperes
 */
double Drive::GetDriveCurrent() const {
    return (Util::abs(m_rightMotor->GetOutputCurrent()) +
            Util::abs(m_leftMotor->GetOutputCurrent())) / 2.0;
}

/**
 * Returns calculated current angle thorugh gyro translation
 *
 * @return  Current angle position with respect to initial position
 */
double Drive::GetAngle() const {
    return -(m_angle - m_gyroZero);
}

/**
 * Returns calculated current anglular rate thorugh gyro translation
 *
 * @return  Current angular rate
 */
double Drive::GetAngularRate() const {
    return -m_angleRate;
}

/**
 * Calculates Drive Output and sets it from driver input or closed control loop
 *
 * @param left  desired left Output
 * @param right desired right Output
 */
void Drive::SetDriveOutput(double left, double right) {
	m_leftCommand = left;
	m_rightCommand = right;

  if (m_controlMode == CANSpeedController::ControlMode::kSpeed) {
      m_leftCommand /= DRIVE_IPS_FROM_RPM;
      m_rightCommand /= DRIVE_IPS_FROM_RPM;
  }
  else if (m_controlMode == CANSpeedController::ControlMode::kPosition) {
      m_leftCommand /= DRIVE_DIST_PER_REVOLUTION;
      m_rightCommand /= DRIVE_DIST_PER_REVOLUTION;
  }

	if (isnan(m_leftCommand) || isnan(m_rightCommand)) {
		m_leftMotor->Set(0.0);
		m_rightMotor->Set(0.0);
	}
	else {
		m_leftMotor->Set(m_leftCommand);
		m_rightMotor->Set(-m_rightCommand);
	}
}

/**
 * Set Drive Talon control modes
 */
void Drive::SetDriveControlMode(CANSpeedController::ControlMode mode){
    m_leftMotor->SetControlMode(mode);
    m_rightMotor->SetControlMode(mode);
    m_controlMode = mode;
}

void Drive::TaskPeriodic(RobotMode mode) {
    m_angle = m_austinGyro->GetAngle();

    //CTRE PigeonImu config
    /*
    double xyz_dps[4];
    m_gyro->GetRawGyro(xyz_dps);
    m_angleRate = xyz_dps[2];
    */

    //Austin ADXRS450_Gyro config
    double currRate = m_austinGyro->GetRate();
    if(currRate == 0){
    }
    else{
      m_angleRate = currRate;
    }

    DBStringPrintf(DB_LINE9, "l %2.1lf r %2.1lf g %2.1lf",
            this->GetLeftDist(),
            this->GetRightDist(),
            this->GetAngle());

    m_angleLog->LogDouble(GetAngle());
    m_angularRateLog->LogDouble(GetAngularRate());

    m_leftDistLog->LogDouble(GetLeftDist());
    m_leftDistRateLog->LogDouble(GetLeftRate());

    m_rightDistLog->LogDouble(GetRightDist());
    m_rightDistRateLog->LogDouble(GetRightRate());

    if (m_controlMode == CANSpeedController::ControlMode::kSpeed) {
        m_leftCommandLog->LogDouble(m_leftCommand * DRIVE_IPS_FROM_RPM);
        m_rightCommandLog->LogDouble(m_rightCommand * DRIVE_IPS_FROM_RPM);
    }
    else if (m_controlMode == CANSpeedController::ControlMode::kPosition) {
        m_leftCommandLog->LogDouble(m_leftCommand * DRIVE_DIST_PER_REVOLUTION);
        m_rightCommandLog->LogDouble(m_rightCommand * DRIVE_DIST_PER_REVOLUTION);
    }
    else {
        m_leftCommandLog->LogDouble(m_leftCommand);
        m_rightCommandLog->LogDouble(m_rightCommand);
    }

    m_leftVoltageLog->LogDouble(m_leftMotor->GetOutputVoltage());
    m_rightVoltageLog->LogDouble(m_rightMotor->GetOutputVoltage());

    m_currentLog->LogDouble(GetDriveCurrent());
}

void Drive::SetBoilerJoystickTerm(double throttle, double turn) {
    m_boilerPixyDriveController->SetJoystickTerm(throttle, turn);
}

void Drive::DriveStraight(RelativeTo relativeTo,
        double dist, double angle) {
    this->SetDriveController(m_straightDriveController);
    m_straightDriveController->SetTarget(relativeTo, dist, angle, this);
}

TrapDriveController *Drive::TrapDrive(RelativeTo relativeTo,
        double dist, double angle) {
    this->SetDriveController(m_trapDriveController);
    m_trapDriveController->SetTarget(relativeTo, dist, angle);
    return m_trapDriveController;
}

SplineDriveController *Drive::SplineDrive(RelativeTo relativeTo,
        double dist, double angle) {
    this->SetDriveController(m_splineDriveController);
    m_splineDriveController->SetTarget(relativeTo, dist, angle);
    return m_splineDriveController;
}

}
