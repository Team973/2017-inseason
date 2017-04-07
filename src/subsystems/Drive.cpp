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
#include "lib/SPIGyro.h"

namespace frc973 {

Drive::Drive(TaskMgr *scheduler, CANTalon *left, CANTalon *right,
            CANTalon *spareTalon,
            LogSpreadsheet *logger, BoilerPixy *boilerPixy, PixyThread *gearPixy,
            ADXRS450_Gyro *gyro
            )
         : DriveBase(scheduler, this, this, nullptr)
         , m_gyro(new PigeonImu(spareTalon))
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

void Drive::Zero() {
    if (m_gyro) {
        m_gyroZero = m_austinGyro->GetAngle();
    }
    if (m_leftMotor) {
        m_leftPosZero = m_leftMotor->GetPosition() * DRIVE_DIST_PER_REVOLUTION;
    }
    if (m_rightMotor) {
        m_rightPosZero = -m_rightMotor->GetPosition() * DRIVE_DIST_PER_REVOLUTION;
    }
}

void Drive::ArcadeDrive(double throttle, double turn) {
    this->SetDriveController(m_arcadeDriveController);
    m_arcadeDriveController->SetJoysticks(throttle, turn);
}

void Drive::OpenloopArcadeDrive(double throttle, double turn) {
    this->SetDriveController(m_openloopArcadeDriveController);
    m_openloopArcadeDriveController->SetJoysticks(throttle, turn);
}

void Drive::AssistedArcadeDrive(double throttle, double turn){
  this->SetDriveController(m_assistedArcadeDriveController);
  m_assistedArcadeDriveController->SetJoysticks(throttle, turn);
}

void Drive::SetBoilerPixyTargeting(){
  printf("got here fam\n");
  this->SetDriveController(m_boilerPixyDriveController);
}

void Drive::SetGearPixyTargeting(){
  this->SetDriveController(m_gearPixyDriveController);
}

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
 * reported in inches
 */
double Drive::GetLeftDist() const {
    return m_leftMotor->GetPosition() * DRIVE_DIST_PER_REVOLUTION -
        m_leftPosZero;
}

double Drive::GetRightDist() const {
    return -m_rightMotor->GetPosition() * DRIVE_DIST_PER_REVOLUTION -
        m_rightPosZero;
}

/**
 * Reported in inches per second
 * As per manual 17.2.1, GetSpeed reports RPM
 */
double Drive::GetLeftRate() const {
    return m_leftMotor->GetSpeed() * DRIVE_IPS_FROM_RPM;
}

double Drive::GetRightRate() const {
    return -m_rightMotor->GetSpeed() * DRIVE_IPS_FROM_RPM;
}

double Drive::GetDist() const {
    return (GetLeftDist() + GetRightDist()) / 2.0;
}

double Drive::GetRate() const {
    return (GetLeftRate() + GetRightRate()) / 2.0;
}

double Drive::GetDriveCurrent() const {
    return (m_rightMotor->GetOutputCurrent() +
            m_leftMotor->GetOutputCurrent()) / 2.0;
}

double Drive::GetAngle() const {
    return -(m_angle - m_gyroZero);
}

double Drive::GetAngularRate() const {
    return -m_angleRate;
}

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

void Drive::SetDriveControlMode(CANSpeedController::ControlMode mode){
    m_leftMotor->SetControlMode(mode);
    m_rightMotor->SetControlMode(mode);
    m_controlMode = mode;
}

void Drive::TaskPeriodic(RobotMode mode) {
    m_angle = m_austinGyro->GetAngle();

    /*
    double xyz_dps[4];
    m_gyro->GetRawGyro(xyz_dps);
    m_angleRate = xyz_dps[2];
    */
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

TrapDriveController *Drive::TrapDrive(RelativeTo relativeTo,
        double dist, double angle) {
    this->SetDriveController(m_trapDriveController);
    m_trapDriveController->SetTarget(relativeTo, dist, angle);
    return m_trapDriveController;
}

}
