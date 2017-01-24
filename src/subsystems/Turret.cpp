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
#include "subsystems/PixyThread.h"

namespace frc973{

constexpr double CLICKS_PER_DEGREE = 14.0;

Turret::Turret(TaskMgr *scheduler, LogSpreadsheet *logger, PixyThread *pixyThread) :
  m_turretMotor(new CANTalon(SHOOTER_TURRET_CAN_ID)),
  m_scheduler(scheduler),
  m_greenFlashlight(new Solenoid(1, 7)),
  m_turretState(TurretState::notRunning),
  m_turretPos(0.0),
  m_turretPower(0.0),
  m_pixyThread(pixyThread)
{
  	m_scheduler->RegisterTask("Turret", this, TASK_PERIODIC);
  	m_turretMotor->SetFeedbackDevice(CANTalon::FeedbackDevice::QuadEncoder);
  	m_turretMotor->SetSensorDirection(true);
  	m_turretMotor->ConfigForwardLimit(80.0 * CLICKS_PER_DEGREE);
  	m_turretMotor->ConfigReverseLimit(-80.0 * CLICKS_PER_DEGREE);
    m_greenFlashlight->Set(true);
}
Turret::~Turret(){
  m_scheduler->UnregisterTask(this);
}

void Turret::SetTurretPosition(double position){
	m_turretMotor->SetControlMode(CANTalon::ControlMode::kPosition);
  m_turretMotor->Set(position * CLICKS_PER_DEGREE);
  m_turretPos = position * CLICKS_PER_DEGREE;
  m_turretState = TurretState::runningPos;
}
double Turret::GetTurretPosition(){
  return m_turretPos;
}
void Turret::StopTurret(){
	m_turretMotor->SetControlMode(CANTalon::ControlMode::kPercentVbus);
  m_turretMotor->Set(0.0);
  m_turretState = TurretState::notRunning;
  m_turretPower = 0.0;
  printf("Setting turret to stop\n");
}

void Turret::SetTurretPower(double power){
	m_turretMotor->SetControlMode(CANTalon::ControlMode::kPercentVbus);
  m_turretMotor->Set(power);
  m_turretState = TurretState::runningPow;
  m_turretPower = power;
  printf("Setting turret power\n");
}

void Turret::SetTurretAutoTarget(){
  m_turretMotor->SetControlMode(CANTalon::ControlMode::kPercentVbus);
  m_turretState = TurretState::vision;
  printf("Setting turret to vision\n");
}
void Turret::TaskPeriodic(RobotMode mode){
  double pixyOffset = m_pixyThread->GetOffset();
  bool pixyDataFresh = m_pixyThread->GetDataFresh();
  printf("PIXY offset: %f, fresh: %d\n", pixyOffset, pixyDataFresh);
  //printf("blocks %d aveX %d\n", m_pixyI2C->GetBlocks(4), m_prevX);

  DBStringPrintf(DB_LINE3, "%.2f encoder mode %d",
          m_turretMotor->GetPosition() / CLICKS_PER_DEGREE,
          m_turretState);

  switch (m_turretState) {
    case runningPow:
      m_turretMotor->Set(m_turretPower);
      DBStringPrintf(DB_LINE0, "%f pow", m_turretPower);
      break;
    case runningPos:
      m_turretMotor->Set(m_turretPos);
      break;
    case notRunning:
      m_turretMotor->Set(0.0);
      break;
    case vision:
      if (pixyDataFresh) {
          m_turretMotor->Set(0.3 * pixyOffset);
      }
      else {
          m_turretMotor->Set(0);
      }
      printf("%f turretReading\n", 0.3 * pixyOffset);
      break;
    default:
      m_turretMotor->Set(0.0);
      break;
  }
}
}
