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
#include "lib/PixyI2C.h"

namespace frc973{

constexpr double CLICKS_PER_DEGREE = 14.0;

Turret::Turret(TaskMgr *scheduler, LogSpreadsheet *logger) :
  m_turretMotor(new CANTalon(SHOOTER_TURRET_CAN_ID)),
  m_scheduler(scheduler),
  m_turretState(TurretState::notRunning),
  m_greenFlashlight(new Solenoid(1, 7)),
  m_turretPos(0.0),
  m_pixyI2C(new TPixy(new LinkI2C())),
  m_turretPower(0.0)
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

  static int m_prevX;

  int numBlocks = m_pixyI2C->GetBlocks();
  if (numBlocks >= 2){
    if (m_pixyI2C->blocks[0].x <= m_pixyI2C->blocks[1].x ){
      m_prevX = m_pixyI2C->blocks[0].x;
    }
    else if (m_pixyI2C->blocks[0].x > m_pixyI2C->blocks[1].x) {
      m_prevX = m_pixyI2C->blocks[1].x;
    }
  }
  else if (numBlocks == 1){
    m_prevX = m_pixyI2C->blocks[0].x;
  }
  else if (numBlocks == 0){
  }

  DBStringPrintf(DB_LINE5, "AveX %d", m_prevX);
  DBStringPrintf(DB_LINE6, "numBlocks %d", numBlocks);

  //printf("blocks %d aveX %d\n", m_pixyI2C->GetBlocks(4), m_prevX);

  DBStringPrintf(DB_LINE3, "%.2f encoder mode %d", m_turretMotor->GetPosition() / CLICKS_PER_DEGREE,
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
      m_turretMotor->Set(0.001 * (m_prevX - 160));
      printf("%f turretReading\n", 0.001 * (m_prevX - 160));
      break;
    default:
      m_turretMotor->Set(0.0);
      break;
  }
}
}
