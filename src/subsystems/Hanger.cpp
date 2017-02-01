/*
 * Hanger.cpp
 *
 *  Created on: Apr 17, 2016
 *      Author: Andrew
 */

#include "subsystems/Hanger.h"
#include "lib/WrapDash.h"
#include "lib/TaskMgr.h"

#include "WPILib.h"
#include "RobotInfo.h"
#include "CANTalon.h"

namespace frc973 {
	static double DEFAULT_HANG_POWER = 1.0;
	static constexpr double HANGER_POS_SETPT = 90.0;
	static constexpr double ARM_CURRENT_THRESHOLD = 40.0;

	Hanger::Hanger(TaskMgr *scheduler):
			 CoopTask(),
			 m_scheduler(scheduler),
			 m_crankMotor(new CANTalon(HANGER_CAN_ID)),
			 m_hangerState(HangerState::preHang)
	{
		m_scheduler->RegisterTask("Hanger", this, TASK_PERIODIC);
		m_crankMotor->SetControlMode(CANTalon::ControlMode::kPercentVbus);
		m_crankMotor->EnableCurrentLimit(true);
		m_crankMotor->SetCurrentLimit(40);
	}

	Hanger::~Hanger() {
		m_scheduler->UnregisterTask(this);
	}

	void Hanger::SetHangerState(HangerState hangerState){
		switch (hangerState) {
			case preHang:
				m_hangerState = HangerState::preHang;
				m_crankMotor->SetControlMode(CANTalon::ControlMode::kPercentVbus);
				break;
			case autoHang:
				m_hangerState = HangerState::autoHang;
				break;
			case armed:
				m_hangerState = HangerState::armed;
				m_crankMotor->SetControlMode(CANTalon::ControlMode::kSpeed);
				break;
		}
	}

	void Hanger::TaskPeriodic(RobotMode mode) {
		m_crankCurrent = m_crankMotor->GetOutputCurrent();
		switch (m_hangerState) {
		case preHang:
			m_crankMotor->Set(0.0);
			break;
		case autoHang:
			m_crankMotor->Set(DEFAULT_HANG_POWER);
			break;
		case armed:
			m_crankMotor->Set(HANGER_POS_SETPT);
			if (m_crankCurrent > ARM_CURRENT_THRESHOLD) {
					m_hangerState = HangerState::autoHang;
					m_crankMotor->SetControlMode(CANTalon::ControlMode::kPercentVbus);
				}
			break;
		}
	}

} /* namespace frc973 */
