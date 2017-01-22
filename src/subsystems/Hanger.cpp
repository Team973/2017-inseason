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
	Hanger::Hanger(TaskMgr *scheduler):
			 CoopTask(),
			 m_scheduler(scheduler),
			 m_crankMotor(new CANTalon(HANGER_CAN_ID)),
			 m_ptoRelease(new DoubleSolenoid(POWER_TAKEOFF_SOL_A, POWER_TAKEOFF_SOL_B)),
			 m_hookReleased(false),
			 m_hangerState(HangerState::preHang)
	{
		m_scheduler->RegisterTask("Hanger", this, TASK_PERIODIC);
	}

	Hanger::~Hanger() {
		m_scheduler->UnregisterTask(this);
	}

	static double DEFAULT_HANG_POWER = 1.0;

	void Hanger::SetPreHang(){
		m_crankMotor->SetControlMode(CANTalon::ControlMode::kPercentVbus);
		m_crankMotor->Set(0.0);
		m_hangerState = HangerState::preHang;
	}

	void Hanger::ReleaseAutoHang(){
		if (m_hookReleased != true){
			m_ptoRelease->Set(DoubleSolenoid::Value::kForward);
			m_hookReleased = true;
			m_hangerState = HangerState::autoHang;
		}
		else {
		}
	}

	void Hanger::SetPostHang(){
		m_crankMotor->SetControlMode(CANTalon::ControlMode::kPercentVbus);
		m_crankMotor->Set(DEFAULT_HANG_POWER);
		m_hangerState = HangerState::postHang;
	}

	void Hanger::TaskPeriodic(RobotMode mode) {
	}

} /* namespace frc973 */
