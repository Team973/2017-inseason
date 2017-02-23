#include <stdio.h>

#include "RobotInfo.h"

#include "subsystems/Drive.h"
#include "WPILib.h"
#include "lib/util/Util.h"

#include "lib/filters/RampedOutput.h"

#include "lib/logging/LogSpreadsheet.h"
#include "lib/WrapDash.h"

#include "controllers/ArcadeDriveController.h"
#include "controllers/PIDDrive.h"
#include "controllers/BoilerPixyVisionDriveController.h"
#include "controllers/GearPixyVisionDriveController.h"

namespace frc973 {

Drive::Drive(TaskMgr *scheduler, CANTalon *left, CANTalon *right,
            CANTalon *spareTalon,
            LogSpreadsheet *logger, BoilerPixy *boilerPixy, PixyThread *gearPixy
            )
         : DriveBase(scheduler, this, this, nullptr)
         , m_gyro(new PigeonImu(spareTalon))
         , m_leftCommand(0.0)
         , m_rightCommand(0.0)
         , m_leftMotor(left)
         , m_rightMotor(right)
         , m_controlMode(CANSpeedController::ControlMode::kPercentVbus)
         , m_arcadeDriveController(nullptr)
         , m_spreadsheet(logger)
         , m_boilerPixyDriveController(
                 new BoilerPixyVisionDriveController(boilerPixy))
         , m_gearPixyDriveController(
                 new GearPixyVisionDriveController(gearPixy))
         , m_angleLog(new LogCell("Angle"))
         , m_angularRateLog(new LogCell("Angular Rate"))
         , m_leftDistLog(new LogCell("Left Encoder Distance"))
         , m_leftDistRateLog(new LogCell("Left Encoder Rate"))
         , m_rightDistLog(new LogCell("Left Encoder Distance"))
         , m_rightDistRateLog(new LogCell("Left Encoder Rate"))
         , m_leftCommandLog(new LogCell("Left motor signal (pow or vel)"))
         , m_rightCommandLog(new LogCell("Right motor signal (pow or vel)"))
         , m_leftVoltageLog(new LogCell("Left motor voltage"))
         , m_rightVoltageLog(new LogCell("Right motor voltage"))
{
    fprintf(stderr, "Initializing Drive Subsystem %p\n", this);
    fprintf(stderr, "Survived fprintf yes its up to date\n");

    m_arcadeDriveController = new ArcadeDriveController();
    m_pidDriveController = new PIDDriveController();
    this->SetDriveController(m_arcadeDriveController);
    this->SetDriveControlMode(m_controlMode);

    bool loggingEnabled = true;
    if (loggingEnabled) {
        m_spreadsheet->RegisterCell(m_angleLog);
        m_spreadsheet->RegisterCell(m_angularRateLog);
        m_spreadsheet->RegisterCell(m_leftDistLog);
        m_spreadsheet->RegisterCell(m_leftDistRateLog);
        m_spreadsheet->RegisterCell(m_leftCommandLog);
        m_spreadsheet->RegisterCell(m_rightCommandLog);
    }
    fprintf(stderr, "Enabled spreadsheets\n");

    scheduler->RegisterTask("Drive", this, TASK_PERIODIC);
    fprintf(stderr, "Scheduled task\n");
}

void Drive::Zero() {
    if (m_gyro)
        m_gyro->SetYaw(0.0);
}

void Drive::ArcadeDrive(double throttle, double turn) {
    this->SetDriveController(m_arcadeDriveController);
    m_arcadeDriveController->SetJoysticks(throttle, turn);
}

void Drive::SetBoilerPixyTargeting(){
  printf("got here fam\n");
  this->SetDriveController(m_boilerPixyDriveController);
}

void Drive::SetGearPixyTargeting(){
  this->SetDriveController(m_gearPixyDriveController);
}

void Drive::PIDDrive(double dist, double turn, RelativeTo relativity, double powerCap) {
    this->SetDriveController(m_pidDriveController);
    m_pidDriveController->SetCap(powerCap);
    m_pidDriveController->SetTarget(dist, turn,
            relativity, this);
    m_pidDriveController->EnableDist();
}

void Drive::PIDTurn(double turn, RelativeTo relativity, double powerCap) {
    this->SetDriveController(m_pidDriveController);
    m_pidDriveController->SetCap(powerCap);
    m_pidDriveController->SetTarget(0.0, turn,
            relativity, this);
    m_pidDriveController->DisableDist();
}

/**
 * reported in inches
 */
double Drive::GetLeftDist() {
    return m_leftMotor->GetPosition() * DRIVE_DIST_PER_REVOLUTION;
}

double Drive::GetRightDist() {
    return -m_rightMotor->GetPosition() * DRIVE_DIST_PER_REVOLUTION;
}

/**
 * Reported in inches per second
 * As per manual 17.2.1, GetSpeed reports RPM
 */
double Drive::GetLeftRate() {
    return m_leftMotor->GetSpeed() * DRIVE_IPS_FROM_RPM;
}

double Drive::GetRightRate() {
    return -m_rightMotor->GetSpeed() * DRIVE_IPS_FROM_RPM;
}

double Drive::GetDist() {
    return GetLeftDist();
}

double Drive::GetRate() {
    return GetLeftRate();
}

double Drive::GetDriveCurrent() {
    return (m_rightMotor->GetOutputCurrent() +
            m_leftMotor->GetOutputCurrent()) / 2.0;
}

double Drive::GetAngle() {
    return m_gyro->GetFusedHeading();
}

double Drive::GetAngularRate() {
    double xyz_dps[4];
    m_gyro->GetRawGyro(xyz_dps);
//    printf("a %lf b %lf c %lf\n", xyz_dps[0], xyz_dps[1], xyz_dps[2]);
    return xyz_dps[2];
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
//	DBStringPrintf(DB_LINE0, "gyro r %2.1f p", this->GetAngularRate(),
//            this->GetAngle());
    DBStringPrintf(DB_LINE9, "l %2.1lf %2.1lf l %2.1lf %2.1lf",
            this->GetLeftDist(), this->GetLeftRate(),
            this->GetRightDist(), this->GetRightRate());

    m_angleLog->LogDouble(GetAngle());
    m_angularRateLog->LogDouble(GetAngularRate());

    m_leftDistLog->LogDouble(GetLeftDist());
    m_leftDistRateLog->LogDouble(GetLeftRate());

    m_rightDistLog->LogDouble(GetLeftDist());
    m_rightDistRateLog->LogDouble(GetLeftRate());

    m_leftCommandLog->LogDouble(m_leftCommand);
    m_rightCommandLog->LogDouble(m_rightCommand);

    m_leftVoltageLog->LogDouble(m_leftMotor->GetOutputVoltage());
    m_rightVoltageLog->LogDouble(m_rightMotor->GetOutputVoltage());
}

}
