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
  m_scheduler(scheduler),
  m_turretState(TurretState::notRunning),
  m_turretPos(0.0),
  m_turretPower(0.0),
  m_pixyThread(pixyThread)
{
  	m_scheduler->RegisterTask("Turret", this, TASK_PERIODIC);
}
Turret::~Turret(){
  m_scheduler->UnregisterTask(this);
}

void Turret::SetTurretPosition(double position){
  m_turretPos = position * CLICKS_PER_DEGREE;
  m_turretState = TurretState::runningPos;
}
double Turret::GetTurretPosition(){
  return m_turretPos;
}
void Turret::StopTurret(){
  m_turretState = TurretState::notRunning;
  m_turretPower = 0.0;
  printf("Setting turret to stop\n");
}

void Turret::SetTurretPower(double power){
  m_turretState = TurretState::runningPow;
  m_turretPower = power;
  printf("Setting turret power\n");
}

void Turret::SetTurretAutoTarget(){
  m_turretState = TurretState::vision;
  printf("Setting turret to vision\n");
}

void Turret::TaskPeriodic(RobotMode mode){
  double pixyOffset = m_pixyThread->GetOffset();
  bool pixyDataFresh = m_pixyThread->GetDataFresh();
  //printf("PIXY offset: %f, fresh: %d\n", pixyOffset, pixyDataFresh);
  //printf("blocks %d aveX %d\n", m_pixyI2C->GetBlocks(4), m_prevX);
}

}
