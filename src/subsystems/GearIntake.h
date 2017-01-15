#ifndef GEAR_INTAKE_SUBSYSTEM_H
#define GEAR_INTAKE_SUBSYSTEM_H
#include "WPILib.h"
#include "Solenoid.h"
#include "lib/CoopTask.h"
#include "lib/TaskMgr.h"

using namespace frc;

namespace frc973{
  class TaskMgr;

  class GearIntake : public CoopTask{
    public:
      enum GearIntakeState{
        released,
        grabbed
      };

      GearIntake(TaskMgr *scheduler);
      virtual ~GearIntake();

      void GrabGears();
      void ReleaseGears();

      void TaskPeriodic(RobotMode mode) override;

    private:
      TaskMgr *m_scheduler;
      GearIntakeState m_gearIntakeState;
      Solenoid *m_gearIntakeSol;
  };
}
#endif /*GEAR_INTAKE_SUBSYSTEM_H*/
