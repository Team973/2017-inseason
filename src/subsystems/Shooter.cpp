/*
 * Shooter.cpp
 *
 *  Created on: Oct 9, 2015
 *      Author: Andrew
 */

#include "RobotInfo.h"
#include "lib/util/Util.h"
#include "lib/WrapDash.h"
#include "lib/TaskMgr.h"
#include "lib/logging/LogSpreadsheet.h"
#include "Shooter.h"

namespace frc973 {

Shooter::Shooter(TaskMgr *scheduler, LogSpreadsheet *logger) :
		m_scheduler(scheduler),
		m_flywheelState(FlywheelState::notRunning),
		m_flywheelPow(0.0),
		m_flywheelEncoder(new Encoder(SHOOTER_ENCODER_A_DIN, SHOOTER_ENCODER_B_DIN))
{
	m_scheduler->RegisterTask("Shooter", this, TASK_PERIODIC);
	m_shooterRate = new LogCell("ShooterRate", 32, 0);
	m_shooterPow = new LogCell("ShooterPower", 32, 0);
	logger->RegisterCell(m_shooterRate);
	logger->RegisterCell(m_shooterPow);
}

Shooter::~Shooter() {
	m_scheduler->UnregisterTask(this);
}

void Shooter::SetFlywheelPow(double pow){
	m_flywheelState = FlywheelState::running;
	m_flywheelPow = pow;
}

void Shooter::SetFlywheelStop(){
	m_flywheelPow = 0.0;
	m_flywheelState = FlywheelState::notRunning;
}

double Shooter::GetFlywheelRate(){
	return m_flywheelEncoder->GetRate();
}

void Shooter::TaskPeriodic(RobotMode mode) {
	switch(m_flywheelState){
		case running:
			m_flywheelMotor->Set(m_flywheelPow);
			break;
		case notRunning:
			m_flywheelMotor->Set(0.0);
			break;
	}
}

}
