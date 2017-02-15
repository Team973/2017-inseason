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

	Hanger::Hanger(TaskMgr *scheduler):
			 CoopTask(),
			 m_scheduler(scheduler),
			 m_crankMotor(new CANTalon(HANGER_CAN_ID)),
			 m_crankMotorB(new CANTalon(HANGER_CAN_ID_B)),
			 m_hangerState(HangerState::preHang)
	{
		m_scheduler->RegisterTask("Hanger", this, TASK_PERIODIC);
		m_crankMotor->ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Brake);
		m_crankMotor->SetFeedbackDevice(CANTalon::FeedbackDevice::CtreMagEncoder_Relative);
		m_crankMotor->SetControlMode(CANTalon::ControlMode::kPercentVbus);
		m_crankMotor->EnableCurrentLimit(true);
		m_crankMotor->SetCurrentLimit(40);

		m_crankMotorB->ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Brake);
		m_crankMotorB->SetControlMode(CANTalon::ControlMode::kFollower);
		m_crankMotor->SetInverted(true);

		m_crankMotor->SetClosedLoopOutputDirection(true);
		m_crankMotor->SetSensorDirection(false);
		m_crankMotor->SelectProfileSlot(0);
		m_crankMotor->SetP(0.0035);
		m_crankMotor->SetI(0.0000012);
		m_crankMotor->SetD(0);

		m_crankMotorB->Set(m_crankMotor->GetDeviceID());
	  m_crankMotorB->SetClosedLoopOutputDirection(true);
	}

	Hanger::~Hanger() {
		m_scheduler->UnregisterTask(this);
	}

	void Hanger::SetHangerState(HangerState hangerState){
		switch (hangerState) {
			case start:
				m_hangerState = HangerState::start;
			case preHang:
				m_hangerState = HangerState::preHang;
				m_crankMotor->SetControlMode(CANSpeedController::ControlMode::kPosition);
				break;
			case autoHang:
				m_hangerState = HangerState::autoHang;
				m_crankMotor->SetControlMode(CANSpeedController::ControlMode::kPercentVbus);
				break;
			case armed:
				m_hangerState = HangerState::armed;
				m_crankMotor->SetControlMode(CANTalon::ControlMode::kSpeed);
				break;
		}
	}

	void Hanger::SetHangerClosedLoop(double position){
		m_crankMotor->ConfigPeakOutputVoltage(3, -3);
		m_crankMotor->Set(position);
	}

	void Hanger::TaskPeriodic(RobotMode mode) {
		m_crankCurrent = m_crankMotor->GetOutputCurrent();
		DBStringPrintf(DB_LINE2, "hang %2.1f", m_crankMotor->GetPosition());
		switch (m_hangerState) {
			case start:
				this->SetHangerClosedLoop(0.0);
				break;
			case autoHang:
				m_crankMotor->Set(DEFAULT_HANG_POWER);
				break;
			case armed:
				this->SetHangerClosedLoop(HANGER_POS_SETPT);
				if (m_crankMotor->GetClosedLoopError() > -10) {
						m_crankMotor->ConfigPeakOutputVoltage(12, -12);
						m_crankMotor->SetControlMode(CANSpeedController::ControlMode::kPercentVbus);
						m_hangerState = HangerState::autoHang;
				}
				break;
		}
	}

} /* namespace frc973 */
