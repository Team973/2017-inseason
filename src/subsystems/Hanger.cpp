/*
 * Hanger.cpp
 *
 *  Created on: Apr 17, 2016
 *      Author: Andrew
 */

#include "subsystems/Hanger.h"
#include "lib/WrapDash.h"
#include "lib/TaskMgr.h"
#include "lib/logging/LogSpreadsheet.h"

#include "WPILib.h"
#include "RobotInfo.h"
#include "CANTalon.h"

namespace frc973 {
    Hanger::Hanger(TaskMgr *scheduler, LogSpreadsheet *logger):
             CoopTask(),
             m_scheduler(scheduler),
             m_crankMotor(new CANTalon(HANGER_CAN_ID)),
             m_crankMotorB(new CANTalon(HANGER_CAN_ID_B)),
             m_hangerState(HangerState::start),
             m_hangStateLog(new LogCell("hanger state", 32)),
             m_hangCurrentLog(new LogCell("hanger current amps", 32))
    {
        m_scheduler->RegisterTask("Hanger", this, TASK_PERIODIC);
        m_crankMotor->ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Brake);
        m_crankMotor->SetFeedbackDevice(CANTalon::FeedbackDevice::CtreMagEncoder_Absolute);
        //m_crankMotor->SetControlMode(CANTalon::ControlMode::kPosition);
        m_crankMotor->SetControlMode(CANTalon::ControlMode::kPercentVbus);
        m_crankMotor->EnableCurrentLimit(true);
        m_crankMotor->SetCurrentLimit(40);

        m_crankMotorB->ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Brake);
        m_crankMotorB->SetControlMode(CANTalon::ControlMode::kFollower);
        m_crankMotor->SetInverted(true);

        m_crankMotor->SetClosedLoopOutputDirection(true);
        m_crankMotor->SetSensorDirection(false);
        m_crankMotor->SelectProfileSlot(0);
        m_crankMotor->SetP(0.0001);
        m_crankMotor->SetI(0.0);
        m_crankMotor->SetD(0);

        m_crankMotorB->Set(m_crankMotor->GetDeviceID());
        m_crankMotorB->SetClosedLoopOutputDirection(true);
        m_crankMotorB->EnableCurrentLimit(true);
        m_crankMotorB->SetCurrentLimit(40);
        m_crankMotor->Set(0.0);

        logger->RegisterCell(m_hangStateLog);
        logger->RegisterCell(m_hangCurrentLog);
    }

    Hanger::~Hanger() {
        m_scheduler->UnregisterTask(this);
    }

    /**
     *  Allows drivers to hang
     *
     *  @param hangerState  desired hanger state
     */
    void Hanger::SetHangerState(HangerState hangerState){
        switch (hangerState) {
            case start: //start open loop hanging
                m_hangerState = HangerState::start;
                m_crankMotor->SetControlMode(CANSpeedController::ControlMode::kPercentVbus);
                break;
            case armed: //defined as period for waiting to attach rope
                m_hangerState = HangerState::armed;
                m_crankMotor->ConfigPeakOutputVoltage(3, -3);
                m_crankMotor->EnableCurrentLimit(true);
                m_crankMotor->SetCurrentLimit(10.0);
                m_crankMotor->SetControlMode(CANTalon::ControlMode::kPercentVbus);
                break;
            case autoHang: //hanging period
                m_hangerState = HangerState::autoHang;
                m_crankMotor->EnableCurrentLimit(true);
                m_crankMotor->SetCurrentLimit(30.0);
                m_crankMotor->ConfigPeakOutputVoltage(12, -12);
                m_crankMotor->SetControlMode(CANSpeedController::ControlMode::kPercentVbus);
                break;
        }
    }

    void Hanger::TaskPeriodic(RobotMode mode) {
        m_crankCurrent = m_crankMotor->GetOutputCurrent();
        /*
        DBStringPrintf(DB_LINE2, "hang %d c %2.1f",
                m_hangerState, m_crankCurrent);
        */
        switch (m_hangerState) {
            case start:
                m_crankMotor->Set(0.0);
                break;
            case autoHang:
                m_crankMotor->Set(1.0);
                break;
            case armed:
                m_crankMotor->Set(1.0);
                if (m_crankCurrent > 6) {
                    SetHangerState(HangerState::autoHang);
                }
                break;
        }

        m_hangStateLog->LogInt(m_hangerState);
        m_hangCurrentLog->LogInt(m_crankMotor->GetOutputCurrent());
    }

} /* namespace frc973 */
