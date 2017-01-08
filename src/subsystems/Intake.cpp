/*
 * Intake.cpp
 *
 *  Created on: Oct 9, 2015
 *      Author: Andrew
 */

#include <subsystems/Intake.h>
#include "RobotInfo.h"
#include "lib/TaskMgr.h"
#include "lib/WrapDash.h"

namespace frc973 {

Intake::Intake(TaskMgr *scheduler) :
	m_intakeMotor(new VictorSP(BALL_INTAKE_MOTOR_PWM)),
	m_intakeMotorB(new VictorSP(ARM_MOTOR_PWM)),
	m_intakeSolenoid(new Solenoid(INTAKE_EXTENSION_SOL)),
	m_intakeMode(IntakeMode::off),
	m_intakePosition(IntakePosition::retracted),
	m_scheduler(scheduler)
{
	this->m_scheduler->RegisterTask("Intake", this, TASK_PERIODIC);
}

Intake::~Intake() {
	this->m_scheduler->UnregisterTask(this);
}

void Intake::SetIntakeMode(IntakeMode mode) {
	m_intakeMode = mode;
}

void Intake::SetIntakePosition(IntakePosition newPos) {
	if (newPos != m_intakePosition) {
		switch (newPos) {
		case IntakePosition::extended:
			m_intakeSolenoid->Set(true);
			break;
		case IntakePosition::retracted:
			m_intakeSolenoid->Set(false);
			break;
		}
	}

	m_intakePosition = newPos;
}

void Intake::SetIntakePower(double pow) {
	m_intakeMode = IntakeMode::manual;
	m_intakeMotor->Set(pow);
	m_intakeMotorB->Set(pow);
}

void Intake::TaskPeriodic(RobotMode mode) {
	switch (m_intakeMode) {
	case IntakeMode::off:
		m_intakeMotor->Set(0.0);
		m_intakeMotorB->Set(0.0);
		break;
	case IntakeMode::forward:
		m_intakeMotor->Set(INTAKE_FORWARD_SPEED);
		m_intakeMotorB->Set(-INTAKE_FORWARD_SPEED);
		break;
	case IntakeMode::reverse:
		m_intakeMotor->Set(INTAKE_REVERSE_SPEED);
		m_intakeMotorB->Set(-INTAKE_REVERSE_SPEED);
		break;
	case IntakeMode::manual:
		break;
	}
}

}
