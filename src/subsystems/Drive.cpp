#include <stdio.h>

#include "RobotInfo.h"

#include "subsystems/Drive.h"
#include "WPILib.h"
#include "lib/util/Util.h"

#include "lib/filters/RampedOutput.h"

#include "lib/logging/LogSpreadsheet.h"

#include "controllers/ArcadeDriveController.h"
#include "controllers/PIDDrive.h"

namespace frc973 {

Drive::Drive(TaskMgr *scheduler, CANTalon *left, CANTalon *right,
			CANTalon *spareTalon,
			LogSpreadsheet *logger
			)
		 : DriveBase(scheduler, this, this, nullptr)
		 , m_gyro(new PigeonImu(spareTalon))
		 , m_leftPower(0.0)
		 , m_rightPower(0.0)
		 , m_leftMotor(left)
		 , m_rightMotor(right)
		 , m_leftMotorPowerFilter(dynamic_cast<FilterBase*>(new RampedOutput(10.0)))
		 , m_rightMotorPowerFilter(dynamic_cast<FilterBase*>(new RampedOutput(10.0)))
		 , m_arcadeDriveController(nullptr)
		 , m_spreadsheet(logger)
		 , m_angleLog(new LogCell("Angle"))
		 , m_angularRateLog(new LogCell("Angular Rate"))
		 , m_leftDistLog(new LogCell("Left Encoder Distance"))
		 , m_leftDistRateLog(new LogCell("Left Encoder Rate"))
		 , m_leftPowerLog(new LogCell("Left motor power"))
		 , m_rightPowerLog(new LogCell("Right motor power"))
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

	bool loggingEnabled = true;
	if (loggingEnabled) {
		m_spreadsheet->RegisterCell(m_angleLog);
		m_spreadsheet->RegisterCell(m_angularRateLog);
		m_spreadsheet->RegisterCell(m_leftDistLog);
		m_spreadsheet->RegisterCell(m_leftDistRateLog);
		m_spreadsheet->RegisterCell(m_leftPowerLog);
		m_spreadsheet->RegisterCell(m_rightPowerLog);
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
	m_pidDriveController->SetTarget(dist, turn, relativity, this);
}

double Drive::GetLeftDist() {
	return -m_leftMotor->GetPosition();
}

double Drive::GetRightDist() {
	return m_rightMotor->GetPosition();
}

double Drive::GetLeftRate() {
	return -m_leftMotor->GetSpeed();
}

double Drive::GetRightRate() {
	return m_rightMotor->GetSpeed();
}

double Drive::GetDist() {
	return GetLeftDist();
}

double Drive::GetRate() {
	return GetLeftRate();
}

double Drive::GetAngle() {
	return -m_gyro->GetFusedHeading();
}

double Drive::GetAngularRate() {
	double xyz_dps[3];
	m_gyro->GetRawGyro(xyz_dps);
	printf("a %d b %d c %d d %d\n", xyz_dps[0], xyz_dps[1], xyz_dps[2], xyz_dps[3]);
	return xyz_dps[2];
}

void Drive::SetDriveOutput(double left, double right) {
	m_leftPower = left;
	m_rightPower = right;

	if (isnan(m_leftPower) || isnan(m_rightPower)) {
		m_leftMotor->Set(0.0);
		m_rightMotor->Set(0.0);
	}
	else {
		m_leftMotor->Set(
				m_leftMotorPowerFilter->Update(
						Util::bound(m_leftPower, -1.0, 1.0)));
		m_rightMotor->Set(
				m_rightMotorPowerFilter->Update(
						Util::bound(-m_rightPower, -1.0, 1.0)));
	}
}

void Drive::TaskPeriodic(RobotMode mode) {
}

}
