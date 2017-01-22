#ifndef GEAR_INTAKE_SUBSYSTEM_H
#define GEAR_INTAKE_SUBSYSTEM_H
#include "WPILib.h"
#include "Solenoid.h"
#include "lib/CoopTask.h"
#include "lib/TaskMgr.h"
#include "CANTalon.h"

using namespace frc;

namespace frc973{
  class TaskMgr;

  class GearIntake : public CoopTask{
    public:
      enum GearIntakeState{
        released,
        grabbed,
        floating
      };

      enum GearPosition{
        up,
        down
      };

      enum Indexer {
        intaking,
        holding,
        indexing
      };

      GearIntake(TaskMgr *scheduler);
      virtual ~GearIntake();

      void GrabGears();
      void ReleaseGears();
      void FloatGears();

      void SetIntakeUp();
      void SetIntakeDown();

      void SetIntakingMode();
      void SetHoldingMode();
      void SetIndexingMode();

      bool IsGearAligned();

      void TaskPeriodic(RobotMode mode) override;

    private:
      TaskMgr *m_scheduler;

      GearIntakeState m_gearIntakeState;
      GearPosition m_gearPosition;
      Indexer m_indexer;

      Solenoid *m_gearIntakeGrip;
      Solenoid *m_gearIntakePos;

      CANTalon *m_leftIndexer;
      CANTalon *m_rightIndexer;

      bool  m_bannerSensor;
  };
}
#endif /*GEAR_INTAKE_SUBSYSTEM_H*/
