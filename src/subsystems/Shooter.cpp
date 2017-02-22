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
        m_flywheelSpeedSetpt(0.0),
        m_flywheelOnTargetFilter(0.5)
{
    m_flywheelMotorPrimary->SetFeedbackDevice(CANTalon::FeedbackDevice::CtreMagEncoder_Relative);
    m_flywheelMotorPrimary->ConfigNeutralMode(CANSpeedController::NeutralMode::kNeutralMode_Coast);
    m_flywheelMotorPrimary->SetClosedLoopOutputDirection(false);
    m_flywheelMotorPrimary->ConfigLimitSwitchOverrides(false, false);
    m_flywheelMotorPrimary->SetSensorDirection(false);
    m_flywheelMotorPrimary->SetControlMode(CANSpeedController::ControlMode::kSpeed);
    m_flywheelMotorPrimary->SelectProfileSlot(0);
    m_flywheelMotorPrimary->ConfigNominalOutputVoltage(0, 0);
    m_flywheelMotorPrimary->ConfigPeakOutputVoltage(12, -12);
    m_flywheelMotorPrimary->SetP(0.05);
    m_flywheelMotorPrimary->SetI(0.0);
    m_flywheelMotorPrimary->SetD(4.00);
    m_flywheelMotorPrimary->SetF(0.024);
    //m_flywheelMotorPrimary->SetVelocityMeasurementPeriod(CANTalon::Period_20Ms);
    //m_flywheelMotorPrimary->SetVelocityMeasurementWindow(64);

    m_flywheelMotorReplica->ConfigNeutralMode(
            CANSpeedController::NeutralMode::kNeutralMode_Coast);
    m_flywheelMotorReplica->SetControlMode(
            CANSpeedController::ControlMode::kFollower);
    m_flywheelMotorReplica->Set(m_flywheelMotorPrimary->GetDeviceID());
    m_flywheelMotorReplica->SetClosedLoopOutputDirection(true);

    m_leftAgitator->SetControlMode(CANSpeedController::ControlMode::kPercentVbus);
    m_rightAgitator->SetControlMode(CANSpeedController::ControlMode::kPercentVbus);
    m_ballConveyor->SetControlMode(CANSpeedController::ControlMode::kPercentVbus);

    m_leftAgitator->EnableCurrentLimit(true);
    m_rightAgitator->EnableCurrentLimit(true);
    m_leftAgitator->SetCurrentLimit(20);
    m_rightAgitator->SetCurrentLimit(20);
    m_leftAgitator->SetVoltageRampRate(6.0);
    m_rightAgitator->SetVoltageRampRate(6.0);

    m_scheduler->RegisterTask("Shooter", this, TASK_PERIODIC);
    m_flywheelRate = new LogCell("FlywheelRate", 32);
    m_flywheelPowLog = new LogCell("Flywheel voltage", 32);
    m_flywheelAmpsLog = new LogCell("Flywheel current", 32);
    m_flywheelStateLog = new LogCell("FlywheelState", 32);
    m_speedSetpoint = new LogCell("SpeedSetpoint", 32);
    m_conveyorLog = new LogCell("Conveyor curr", 32);
    m_leftAgitatorLog = new LogCell("leftAgitator curr", 32);
    m_rightAgitatorLog = new LogCell("RightAgitator curr", 32);
    logger->RegisterCell(m_flywheelRate);
    logger->RegisterCell(m_flywheelPowLog);
    logger->RegisterCell(m_speedSetpoint);
    logger->RegisterCell(m_flywheelAmpsLog);
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
    m_flywheelMotorPrimary->Set(m_flywheelSpeedSetpt);
    m_flywheelOnTargetFilter.Update(false);
}

void Shooter::SetFlywheelStop(){
    m_flywheelPow = 0.0;
    m_flywheelMotorPrimary->SetControlMode(CANSpeedController::ControlMode::kPercentVbus);
    m_flywheelMotorPrimary->Set(0.0);
    m_flywheelState = FlywheelState::notRunning;
    m_flywheelOnTargetFilter.Update(false);
}

double Shooter::GetFlywheelRate(){
    return m_flywheelMotorPrimary->GetSpeed();// * (1.0 / 24576.0);
}

bool Shooter::OnTarget() {
    return m_flywheelOnTargetFilter.Update(
            abs(GetFlywheelRate() - m_flywheelSpeedSetpt) < 200.0);
}

void Shooter::StartConveyor(double speed) {
    m_ballConveyor->Set(speed);
    printf("%lf pow on %d - conveyor\n", speed, BALL_CONVEYOR_CAN_ID);
    //DBStringPrintf(DB_LINE3, "conv pow %lf", speed);
}

void Shooter::StopConveyor() {
    m_ballConveyor->Set(0.0);
    printf("%lf pow on %d - conveyor\n", 0.0, BALL_CONVEYOR_CAN_ID);
    //DBStringPrintf(DB_LINE3, "conv pow %lf", 0.0);
}

//side: true = right; false = left
void Shooter::StartAgitator(double speed, bool side){
    if (side == false) {
        m_leftAgitator->Set(speed);
        printf("%lf pow on %d - left agitator\n", speed, LEFT_AGITATOR_CAN_ID);
    }
    else if (side == true) {
        m_rightAgitator->Set(-speed);
        printf("%lf pow on %d - right agitator\n", speed, RIGHT_AGITATOR_CAN_ID);
    }
}

void Shooter::StopAgitator(){
    m_leftAgitator->Set(0.0);
    m_rightAgitator->Set(0.0);
}

void Shooter::TaskPeriodic(RobotMode mode) {
    m_flywheelRate->LogDouble(GetFlywheelRate());
    m_flywheelPowLog->LogDouble(m_flywheelMotorPrimary->GetOutputVoltage());
    m_flywheelAmpsLog->LogDouble(m_flywheelMotorPrimary->GetOutputCurrent());
    m_flywheelStateLog->LogPrintf("%d", m_flywheelState);
    m_speedSetpoint->LogDouble(m_flywheelSpeedSetpt);
    m_conveyorLog->LogDouble(m_ballConveyor->GetOutputCurrent());
    m_leftAgitatorLog->LogDouble(m_leftAgitator->GetOutputCurrent());
    m_rightAgitatorLog->LogDouble(m_rightAgitator->GetOutputCurrent());
    //DBStringPrintf(DB_LINE5,"shooterrate %2.1lf", GetFlywheelRate());
    //DBStringPrintf(DB_LINE6,"shootersetpt %2.1lf", m_flywheelSpeedSetpt);
    //DBStringPrintf(DB_LINE3,"conv %2.1lf flail %2.1lf %2.1lf", m_ballConveyor->GetOutputVoltage(),
                //  m_leftAgitator->GetOutputVoltage(), m_rightAgitator->GetOutputVoltage());
    printf("setpt %lf speed %lf\n",
            m_flywheelSpeedSetpt, GetFlywheelRate());
    //DBStringPrintf(DB_LINE8,"shooterpow %2.1lf", m_flywheelMotorPrimary->GetOutputVoltage());
    switch(m_flywheelState){
        case power:
            m_flywheelMotorPrimary->Set(m_flywheelPow);
            break;
        case notRunning:
            m_flywheelMotorPrimary->Set(0.0);
            break;
        case speed:
            //m_flywheelMotorPrimary->Set(m_flywheelSpeedSetpt);
            break;
    }
}

}
