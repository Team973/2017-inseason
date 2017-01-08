/*
 * Shooter.cpp
 *
 *  Created on: Oct 9, 2015
 *      Author: Andrew
 */

#include "subsystems/Shooter.h"
#include "RobotInfo.h"
#include "lib/util/Util.h"
#include "lib/WrapDash.h"
#include "controllers/ShooterGains_front.h"
#include "controllers/ShooterGains_back.h"
#include "lib/TaskMgr.h"
#include "lib/filters/BullshitFilter.h"
#include "lib/filters/MovingAverageFilter.h"
#include "lib/filters/MedianFilter.h"
#include "lib/filters/CascadingFilter.h"
#include "lib/filters/DelaySwitch.h"
#include "lib/filters/PID.h"
#include "lib/logging/LogSpreadsheet.h"
#include "controllers/StateSpaceFlywheelController.h"

namespace frc973 {

static constexpr bool LONG_SOL_EXTENDED = false;
static constexpr bool LONG_SOL_RETRACTED = true;

static constexpr bool SHORT_SOL_EXTENDED = true;
static constexpr bool SHORT_SOL_RETRACTED = false;

Shooter::Shooter(TaskMgr *scheduler, LogSpreadsheet *logger, VictorSP *conveyorMotor) :
		m_frontFlywheelMotor(new VictorSP(FRONT_SHOOTER_PWM)),
		m_backFlywheelMotor(new VictorSP(BACK_SHOOTER_PWM)),
		m_conveyor(conveyorMotor),
		m_frontFlywheelEncoder(new Counter(FLYWHEEL_FRONT_BANNERSENSOR_DIN)),
		m_backFlywheelEncoder(new Counter(FLYWHEEL_BACK_BANNERSENSOR_DIN)),
		m_frontFlywheelState(FlywheelState::openLoop),
		m_backFlywheelState(FlywheelState::openLoop),
		m_flywheelEnabled(false),
		m_frontFlywheelTargetSpeed(0.0),
		m_backFlywheelTargetSpeed(0.0),
		m_frontController(new StateSpaceFlywheelController(FlywheelGainsBack::MakeGains())),
		m_backController(new StateSpaceFlywheelController(FlywheelGainsBack::MakeGains())),
		m_frontFlywheelSetPower(0.0),
		m_backFlywheelSetPower(0.0),
		m_flywheelReady(false),
		m_frontFilter(new CascadingFilter()),
		m_frontMovingAvgFilt(new MovingAverageFilter(0.85)),
		m_oldFSpeed(0.0),
		m_fmedfilt(new MedianFilter()),
		m_backFilter(new CascadingFilter()),
		m_readyFilter(new DelaySwitch(0.9)),
		m_elevatorState(ElevatorHeight::midHigh),
		m_longSolenoid(new Solenoid(SHOOTER_ANGLE_UPPER_SOL)),
		m_shortSolenoid(new Solenoid(SHOOTER_ANGLE_LOWER_SOL)),
		m_frontFlywheelSpeed(new LogCell("front shooter speed (RPM)")),
		m_frontFlywheelFilteredSpeed(new LogCell("front shooter filtered speed (RPM)")),
		m_shooterPow(new LogCell("shooter power")),
		m_shooterTime(new LogCell("Shooter Time (ms)")),
		m_scheduler(scheduler),
		m_runningLight(new Solenoid(RUNNING_LIGHT_SOL)),
		m_readyLight(new Solenoid(READY_LIGHT_SOL)),
		m_conveyorControl(true)
{
	m_scheduler->RegisterTask("Shooter", this, TASK_PERIODIC);

	m_frontFilter->PushFilter(new BullshitFilter(BullshitFilter::MinBehavior::noMin, 0.0,
			BullshitFilter::MaxBehavior::dropMax, 9000.0));
	m_frontFilter->PushFilter(new MovingAverageFilter(0.85));
	m_frontFilter->PushFilter(new MedianFilter());

	m_backFilter->PushFilter(new BullshitFilter(BullshitFilter::MinBehavior::noMin, 0.0,
			BullshitFilter::MaxBehavior::dropMax, 9000.0));
	m_backFilter->PushFilter(new MovingAverageFilter(0.85));
	m_backFilter->PushFilter(new MedianFilter());

	if (logger != nullptr) {
		logger->RegisterCell(m_shooterPow);
		logger->RegisterCell(m_frontFlywheelSpeed);
		logger->RegisterCell(m_frontFlywheelFilteredSpeed);
		//logger->RegisterCell(m_shooterTime);
	}
}

Shooter::~Shooter() {
	m_scheduler->UnregisterTask(this);
}

void Shooter::SetFlywheelEnabled(bool enabledP) {
	m_flywheelEnabled = enabledP;
	m_runningLight->Set(enabledP);
}

void Shooter::SetFrontFlywheelSSShoot(double goal) {
	m_frontFlywheelState = FlywheelState::ssControl;
	m_frontFlywheelTargetSpeed = goal;
	m_frontController->SetVelocityGoal(m_frontFlywheelTargetSpeed * Constants::PI / 30.0);
}

void Shooter::SetBackFlywheelSSShoot(double goal) {
	m_backFlywheelState = FlywheelState::ssControl;
	m_backFlywheelTargetSpeed = goal;
	m_backController->SetVelocityGoal(m_backFlywheelTargetSpeed * Constants::PI / 30.0);
}

void Shooter::SetFrontFlywheelPower(double pow) {
	m_frontFlywheelState = FlywheelState::openLoop;
	m_frontFlywheelSetPower = pow;
}

void Shooter::SetBackFlywheelPower(double pow) {
	m_backFlywheelState = FlywheelState::openLoop;
	m_backFlywheelSetPower = pow;
}

void Shooter::SetFlywheelStop() {
	m_frontFlywheelState = FlywheelState::openLoop;
	m_backFlywheelState = FlywheelState::openLoop;
	m_frontFlywheelSetPower = 0.0;
	m_backFlywheelSetPower = 0.0;
	printf("stop\n");
}

void Shooter::TaskPeriodic(RobotMode mode) {
	double frontMotorOutput;
	double backMotorOutput;

	if (m_flywheelEnabled) {
		switch(m_frontFlywheelState) {
		case FlywheelState::ssControl:
			frontMotorOutput = m_frontController->Update(
					this->GetFrontFlywheelFilteredRate() * Constants::PI / 30.0);
			m_flywheelReady =
					m_readyFilter->Update(
							Util::abs(this->GetFrontFlywheelFilteredRate() - m_frontFlywheelTargetSpeed) < 50);
			break;
		case FlywheelState::openLoop:
			frontMotorOutput = m_frontFlywheelSetPower;
			m_flywheelReady = false;
			m_readyFilter->Update(false);
			break;
		}
	}
	else {
		m_flywheelReady = false;
		m_readyFilter->Update(false);
		frontMotorOutput = 0.0;
	}

	m_frontFlywheelMotor->Set(frontMotorOutput);
	m_readyLight->Set(m_flywheelReady);

	if (m_flywheelEnabled) {
		switch(m_backFlywheelState) {
		case FlywheelState::ssControl:
			backMotorOutput = m_backController->Update(
					this->GetRearFlywheelFilteredRate() * Constants::PI / 30.0);
			break;
		case FlywheelState::openLoop:
			backMotorOutput = m_backFlywheelSetPower;
			break;
		}
	}
	else {
		backMotorOutput = 0.0;
	}

	m_backFlywheelMotor->Set(backMotorOutput);

	DBStringPrintf(DBStringPos::DB_LINE2,
			"f-rpm %4.0lf pow %1.2lf", GetFrontFlywheelRate(), frontMotorOutput);
	DBStringPrintf(DBStringPos::DB_LINE3,
			"r-rpm %4.0lf pow %1.2lf", GetRearFlywheelRate(), backMotorOutput);

	m_shooterPow->LogDouble(frontMotorOutput);
	m_frontFlywheelSpeed->LogDouble(GetFrontFlywheelRate());
	m_frontFlywheelFilteredSpeed->LogDouble(GetFrontFlywheelFilteredRate());
	m_shooterTime->LogPrintf("%ld", (long) GetUsecTime());
}
/**

 * GetFlywheelRate takes raw flywheel rate data and filters it becauze it's jittery
 *
 * There are 3 types of filtering happening
 *  * If we read something more than 6000.0, it must be false... cap it at 6000.0
 *  * If we read anything more than 1000 times our previous value, it must be false... ignore it
 *  * If this value looks good, we do a moving average filter on it
 */
double Shooter::GetFrontFlywheelRate(void) {
	return (1.0 / m_frontFlywheelEncoder->GetPeriod()) * 60.0;
}

double Shooter::GetFrontFlywheelFilteredRate(void) {
	double s = GetFrontFlywheelRate();
	if (s > 9000.0) {
		s = m_oldFSpeed;
	}
	else {
		s = m_fmedfilt->Update(s);
		s = m_frontMovingAvgFilt->Update(s);
		m_oldFSpeed = s;
	}
	return s;
}

double Shooter::GetRearFlywheelRate(void) {
	return (1.0 / m_backFlywheelEncoder->GetPeriod()) * 60.0;
}

double Shooter::GetRearFlywheelFilteredRate(void) {
	double s = GetRearFlywheelRate();
	return m_backFilter->Update(s);
}

void Shooter::SetElevatorHeight(ElevatorHeight newHeight) {
	if (newHeight != m_elevatorState) {
		m_elevatorState = newHeight;

		switch (m_elevatorState) {
		case ElevatorHeight::wayHigh:
			m_longSolenoid->Set(LONG_SOL_EXTENDED);
			m_shortSolenoid->Set(SHORT_SOL_EXTENDED);
			break;
		case ElevatorHeight::midHigh:
			m_longSolenoid->Set(LONG_SOL_EXTENDED);
			m_shortSolenoid->Set(SHORT_SOL_RETRACTED);
			break;
		case ElevatorHeight::midLow:
			m_longSolenoid->Set(LONG_SOL_RETRACTED);
			m_shortSolenoid->Set(SHORT_SOL_EXTENDED);
			break;
		case ElevatorHeight::wayLow:
			m_longSolenoid->Set(LONG_SOL_RETRACTED);
			m_shortSolenoid->Set(SHORT_SOL_RETRACTED);
			break;
		}
	}
}

}
