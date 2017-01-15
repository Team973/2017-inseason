#include "GearIntake.h"
#include "RobotInfo.h"

namespace frc973{
  GearIntake::GearIntake(TaskMgr *scheduler) :
  m_scheduler(scheduler),
  m_gearIntakeSol(new Solenoid(GEAR_INTAKE_SOL)),
  m_gearIntakeState(GearIntakeState::released)
  {
    this->m_scheduler->RegisterTask("GearIntake", this, TASK_PERIODIC);
  }

  GearIntake::~GearIntake(){
    m_scheduler->UnregisterTask(this);
  }

  void GearIntake::GrabGears(){
    m_gearIntakeSol->Set(false);
    m_gearIntakeState = GearIntakeState::grabbed;
  }

  void GearIntake::ReleaseGears(){
    m_gearIntakeSol->Set(true);
    m_gearIntakeState  = GearIntakeState::released;
  }

  void GearIntake::TaskPeriodic(RobotMode mode){
    }
}
