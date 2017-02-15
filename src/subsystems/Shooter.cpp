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
#include "subsystems/Shooter.h"

namespace frc973 {

Shooter::Shooter(TaskMgr *scheduler, LogSpreadsheet *logger, CANTalon *leftAgitator) :
        m_scheduler(scheduler),
        m_flywheelState(FlywheelState::notRunning),
        m_flywheelMotorPrimary(new CANTalon(FLYWHEEL_PRIMARY_CAN_ID,
                                            FLYWHEEL_CONTROL_PERIOD_MS)),
        m_flywheelMotorReplica(new CANTalon(FLYWHEEL_REPLICA_CAN_ID)),
        m_leftAgitator(leftAgitator),
        m_rightAgitator(new CANTalon(RIGHT_AGITATOR_CAN_ID, 50)),
        m_ballConveyor(new CANTalon(BALL_CONVEYOR_CAN_ID, 50)),
        m_flywheelPow(0.0),
    m_flywheelSpeedSetpt(0.0)
{
    m_flywheelMotorPrimary->SetFeedbackDevice(CANTalon::FeedbackDevice::CtreMagEncoder_Relative);
    m_flywheelMotorPrimary->ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Coast);
    m_flywheelMotorPrimary->SetClosedLoopOutputDirection(true);
    m_flywheelMotorPrimary->SetSensorDirection(false);
    m_flywheelMotorPrimary->SetControlMode(CANSpeedController::ControlMode::kSpeed);
    m_flywheelMotorPrimary->SelectProfileSlot(0);
    m_flywheelMotorPrimary->ConfigNominalOutputVoltage(0, 0);
    m_flywheelMotorPrimary->ConfigPeakOutputVoltage(12, -12);
    m_flywheelMotorPrimary->SetP(0.035);
    m_flywheelMotorPrimary->SetI(0.0000012);
    m_flywheelMotorPrimary->SetD(0);
    m_flywheelMotorPrimary->SetF(1023.0 / 32768.0 * 0.85);

    m_flywheelMotorReplica->ConfigNeutralMode(
            CANSpeedController::NeutralMode::kNeutralMode_Coast);
    m_flywheelMotorReplica->SetControlMode(
            CANSpeedController::ControlMode::kFollower);
    m_flywheelMotorReplica->Set(m_flywheelMotorPrimary->GetDeviceID());
  m_flywheelMotorReplica->SetClosedLoopOutputDirection(false);

    m_leftAgitator->SetControlMode(CANSpeedController::ControlMode::kPercentVbus);
    m_rightAgitator->SetControlMode(CANSpeedController::ControlMode::kPercentVbus);
    m_ballConveyor->SetControlMode(CANSpeedController::ControlMode::kPercentVbus);

    m_scheduler->RegisterTask("Shooter", this, TASK_PERIODIC);
    m_flywheelRate = new LogCell("FlywheelRate", 32);
    m_flywheelPowLog = new LogCell("FlywheelPower", 32);
    m_flywheelStateLog = new LogCell("FlywheelState", 32);
    m_speedSetpoint = new LogCell("SpeedSetpoint", 32);
    logger->RegisterCell(m_flywheelRate);
    logger->RegisterCell(m_flywheelPowLog);
}

Shooter::~Shooter() {
    m_scheduler->UnregisterTask(this);
}

void Shooter::SetFlywheelPow(double pow){
    m_flywheelMotorPrimary->SetControlMode(CANSpeedController::ControlMode::kPercentVbus);
    m_flywheelState = FlywheelState::power;
    m_flywheelPow = pow;
}

void Shooter::SetFlywheelSpeed(double speed){
    m_flywheelMotorPrimary->SetControlMode(CANSpeedController::ControlMode::kSpeed);
    m_flywheelState = FlywheelState::speed;
  m_flywheelSpeedSetpt = speed;
}

void Shooter::SetFlywheelStop(){
    m_flywheelPow = 0.0;
    m_flywheelState = FlywheelState::notRunning;
}

double Shooter::GetFlywheelRate(){
    return m_flywheelMotorPrimary->GetSpeed();// * (1.0 / 24576.0);
}

void Shooter::StartConveyor(double speed) {
    m_ballConveyor->Set(speed);
}

void Shooter::StopConveyor() {
    m_ballConveyor->Set(0.0);
}

//side: true = right; false = left
void Shooter::StartAgitator(double speed, bool side){
    if (side == false) {
        m_leftAgitator->Set(speed);
    }
    else if (side == true) {
        m_rightAgitator->Set(-speed);
    }
}

void Shooter::StopAgitator(){
        m_leftAgitator->Set(0.0);
        m_rightAgitator->Set(0.0);
}

void Shooter::TaskPeriodic(RobotMode mode) {
    m_flywheelRate->LogDouble(GetFlywheelRate());
    m_flywheelPowLog->LogDouble(m_flywheelMotorPrimary->GetOutputVoltage());
    m_flywheelStateLog->LogPrintf("%d", m_flywheelState);
    m_speedSetpoint->LogDouble(DEFAULT_FLYWHEEL_SPEED_SETPOINT);
    DBStringPrintf(DB_LINE5,"shooterrate %2.1f", GetFlywheelRate());
    DBStringPrintf(DB_LINE6,"shootersetpt %2.1f", m_flywheelSpeedSetpt);
    DBStringPrintf(DB_LINE8,"shooterpos %2.1f", m_flywheelMotorPrimary->GetOutputVoltage());
    switch(m_flywheelState){
        case power:
            m_flywheelMotorPrimary->Set(m_flywheelPow);
            break;
        case notRunning:
            m_flywheelMotorPrimary->Set(0.0);
            break;
        case speed:
            m_flywheelMotorPrimary->Set(m_flywheelSpeedSetpt);
            break;
    }
}

}
