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
    m_gearIntakeState = GearIntakeState::grabbed;
  }

  void GearIntake::ReleaseGears(){
    m_gearIntakeState  = GearIntakeState::released;
  }

  void GearIntake::TaskPeriodic(RobotMode mode){
    switch (m_gearIntakeState) {
      case released:
        m_gearIntakeSol->Set(false);
      break;
      case grabbed:
        m_gearIntakeSol->Set(true);
      break;
    }
  }
}
