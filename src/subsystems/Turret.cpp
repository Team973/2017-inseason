/**
  * Turret.cpp and .h
  * Created January 9, 2017
  */
#include "RobotInfo.h"
#include "lib/util/Util.h"
#include "lib/WrapDash.h"
#include "lib/TaskMgr.h"
#include "lib/logging/LogSpreadsheet.h"
#include "subsystems/Turret.h"
#include "CANTalon.h"

namespace frc973{

  constexpr double CLICKS_PER_DEGREE = 14.0;

  Turret::Turret(TaskMgr *scheduler, LogSpreadsheet *logger) :
    m_turretMotor(new CANTalon(SHOOTER_TURRET_CAN_ID)),
    m_scheduler(scheduler),
    m_turretState(TurretState::notRunning),
    m_turretPos(0.0),
    m_turretPower(0.0)
  {
    	m_scheduler->RegisterTask("Turret", this, TASK_PERIODIC);
    	m_turretMotor->SetFeedbackDevice(CANTalon::FeedbackDevice::QuadEncoder);
    	m_turretMotor->SetSensorDirection(true);
    	m_turretMotor->ConfigForwardLimit(80.0 * CLICKS_PER_DEGREE);
    	m_turretMotor->ConfigReverseLimit(-80.0 * CLICKS_PER_DEGREE);
  }
  Turret::~Turret(){
    m_scheduler->UnregisterTask(this);
  }

  void Turret::SetTurretPosition(double position){
  	m_turretMotor->SetControlMode(CANTalon::ControlMode::kPosition);
    m_turretMotor->Set(position);
    m_turretState = TurretState::running;
  }
  double Turret::GetTurretPosition(){
    return m_turretPos;
  }
  void Turret::StopTurret(){
  	m_turretMotor->SetControlMode(CANTalon::ControlMode::kPercentVbus);
    m_turretMotor->Set(0.0);
    m_turretPower = 0.0;
  }
  void Turret::SetTurretPower(double power){
  	m_turretMotor->SetControlMode(CANTalon::ControlMode::kPercentVbus);
    m_turretMotor->Set(power);
    m_turretPower = power;
  }
  void Turret::SetTurretMode(TurretState turretState){
    m_turretState = turretState;
  }
  void Turret::SetTurretAutoTarget(){
    m_turretState = TurretState::vision;
  }
  void Turret::TaskPeriodic(){
    switch (m_turretState) {
      case running:
        break;
      case notRunning:
        m_turretMotor->Set(0.0);
        break;
      case vision:
        break;
      default:
        m_turretMotor->Set(0.0);
        break;
    }
  }
}
