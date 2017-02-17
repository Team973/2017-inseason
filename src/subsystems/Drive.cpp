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

namespace frc973 {

Drive::Drive(TaskMgr *scheduler, CANTalon *left, CANTalon *right,
            CANTalon *spareTalon,
            LogSpreadsheet *logger
            )
         : DriveBase(scheduler, this, this, nullptr)
         , m_gyro(new PigeonImu(spareTalon))
         , m_leftSignal(0.0)
         , m_rightSignal(0.0)
         , m_leftMotor(left)
         , m_rightMotor(right)
         , m_arcadeDriveController(nullptr)
         , m_spreadsheet(logger)
         , m_angleLog(new LogCell("Angle"))
         , m_angularRateLog(new LogCell("Angular Rate"))
         , m_leftDistLog(new LogCell("Left Encoder Distance"))
         , m_leftDistRateLog(new LogCell("Left Encoder Rate"))
         , m_leftSignalLog(new LogCell("Left motor signal (pow or vel)"))
         , m_rightSignalLog(new LogCell("Right motor signal (pow or vel)"))
{
    fprintf(stderr, "Initializing Drive Subsystem %p\n", m_leftEncoder);
    fprintf(stderr, "Survived fprintf yes its up to date\n");

    if (m_leftEncoder != nullptr) {
        fprintf(stderr, "Operating on left drive encoder\n");
        m_leftEncoder->SetDistancePerPulse(1.0);
    }

    m_arcadeDriveController = new ArcadeDriveController();
    m_pidDriveController = new PIDDriveController();
    this->SetDriveController(m_arcadeDriveController);
    //this->SetDriveControlMode(CANSpeedController::ControlMode::kPercentVbus);

    bool loggingEnabled = true;
    if (loggingEnabled) {
        m_spreadsheet->RegisterCell(m_angleLog);
        m_spreadsheet->RegisterCell(m_angularRateLog);
        m_spreadsheet->RegisterCell(m_leftDistLog);
        m_spreadsheet->RegisterCell(m_leftDistRateLog);
        m_spreadsheet->RegisterCell(m_leftSignalLog);
        m_spreadsheet->RegisterCell(m_rightSignalLog);
    }

    scheduler->RegisterTask("Drive", this, TASK_PERIODIC);
}

void Drive::Zero() {
    if (m_leftEncoder)
        m_leftEncoder->Reset();
    if (m_rightEncoder)
        m_rightEncoder->Reset();
    if (m_gyro)
        m_gyro->SetFusedHeading(0.0);
    m_leftEncoder->SetDistancePerPulse(1.0);
}

void Drive::ArcadeDrive(double throttle, double turn) {
    this->SetDriveController(m_arcadeDriveController);
    m_arcadeDriveController->SetJoysticks(throttle, turn);
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

double Drive::GetLeftDist() {
    return m_leftMotor->GetPosition() * DRIVE_DIST_PER_REVOLUTION;
}

double Drive::GetRightDist() {
    return -m_rightMotor->GetPosition() * DRIVE_DIST_PER_REVOLUTION;
}

double Drive::GetLeftRate() {
    return m_leftMotor->GetSpeed() * DRIVE_DIST_PER_REVOLUTION;
}

double Drive::GetRightRate() {
    return -m_rightMotor->GetSpeed() * DRIVE_DIST_PER_REVOLUTION;
}

double Drive::GetDist() {
    return GetLeftDist();
}

double Drive::GetRate() {
    return GetLeftRate();
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
	m_leftSignal = left;
	m_rightSignal = right;

	if (isnan(m_leftSignal) || isnan(m_rightSignal)) {
		m_leftMotor->Set(0.0);
		m_rightMotor->Set(0.0);
	}
	else {
		m_leftMotor->Set(m_leftSignal);
		m_rightMotor->Set(-m_rightSignal);
	}
}

void Drive::SetDriveControlMode(CANSpeedController::ControlMode mode){
    m_leftMotor->SetControlMode(mode);
    m_rightMotor->SetControlMode(mode);
}

void Drive::TaskPeriodic(RobotMode mode) {
	DBStringPrintf(DB_LINE0, "gyro r %2.1f p", this->GetAngularRate(),
            this->GetAngle());
    DBStringPrintf(DB_LINE9, "l %2.1lf %2.1lf l %2.1lf %2.1lf", 
            this->GetLeftDist(), this->GetLeftRate(),
            this->GetRightDist(), this->GetRightRate());
}

}
