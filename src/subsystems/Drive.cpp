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

Drive::Drive(TaskMgr *scheduler,  SpeedController *left, SpeedController *right,
			Encoder *leftEncoder,
            Encoder *rightEncoder,
			Encoder *gyro,
			LogSpreadsheet *logger
			)
		 : DriveBase(scheduler, this, this, nullptr)
		 , m_leftEncoder(leftEncoder)
		 , m_rightEncoder(rightEncoder)
		 , m_gyro(gyro)
		 , m_gearing(DriveGearing::LowGear)
		 , m_gearingSolenoid(new Solenoid(DRIVE_SHIFT_SOL))
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

	scheduler->RegisterTask("DriveBase", this, TASK_PERIODIC);
}

void Drive::SetGearing(DriveGearing newGearing) {
	if (newGearing != m_gearing) {
		switch (newGearing) {
		case DriveGearing::HighGear:
			m_gearingSolenoid->Set(true);
			break;
		case DriveGearing::LowGear:
			m_gearingSolenoid->Set(false);
		}
		m_gearing = newGearing;
	}
}

void Drive::Zero() {
	if (m_leftEncoder)
		m_leftEncoder->Reset();
	if (m_rightEncoder)
		m_rightEncoder->Reset();
	if (m_gyro)
		m_gyro->Reset();
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
	return -m_leftEncoder->Get() * 24.5 / 360.0 * 0.95;
}

double Drive::GetRightDist() {
	printf("Someone didn't get the memo this robot only has one encoder\n");
	return -GetLeftDist();
}

double Drive::GetLeftRate() {
	return m_leftEncoder->GetRate();
}

double Drive::GetRightRate() {
	printf("someone didn't get the memo this robot only has one encoder\n");
	return GetLeftRate();
}

double Drive::GetDist() {
	return GetLeftDist();
}

double Drive::GetRate() {
	return GetLeftRate();
}

double Drive::GetAngle() {
	return -m_gyro->Get();
}

double Drive::GetAngularRate() {
	return -m_gyro->GetRate();
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
						Util::bound(-m_leftPower, -1.0, 1.0)));
		m_rightMotor->Set(
				m_rightMotorPowerFilter->Update(
						Util::bound(-m_rightPower, -1.0, 1.0)));
	}
}

void Drive::TaskPeriodic(RobotMode mode) {
}

}
