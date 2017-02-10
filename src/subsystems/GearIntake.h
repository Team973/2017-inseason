#ifndef GEAR_INTAKE_SUBSYSTEM_H
#define GEAR_INTAKE_SUBSYSTEM_H
#include "WPILib.h"
#include "Solenoid.h"
#include "lib/CoopTask.h"
#include "lib/TaskMgr.h"
#include "CANTalon.h"
#include "lib/WrapDash.h"

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
        indexing,
        stop
      };

      enum PickUp {
        seeking,
        chewing,
        digesting,
        vomiting,
        postVomit,
        manual
      };

      GearIntake(TaskMgr *scheduler);
      virtual ~GearIntake();

      void StartPickupSequence();
      void ReleaseGear();

      void SetGearIntakeState(GearIntakeState gearIntakeState);
      void SetGearPos(GearPosition gearPosition);
      void SetIndexerMode(Indexer indexerMode);
      void SetReleaseAutoEnable(bool driverInput);
      bool IsGearReady();

      void TaskPeriodic(RobotMode mode) override;

    private:

      TaskMgr *m_scheduler;

      GearIntakeState m_gearIntakeState;
      GearPosition m_gearPosition;
      Indexer m_indexer;
      PickUp m_pickUpState;

      Solenoid *m_gearIntakeRelease;
      Solenoid *m_gearIntakeGrab;
      Solenoid *m_gearIntakePos;
      DigitalInput  *m_pushTopLeft;
      DigitalInput  *m_pushTopRight;
      DigitalInput  *m_pushBottom;

      CANTalon *m_leftIndexer;
      CANTalon *m_rightIndexer;

      uint32_t m_gearTimer;
      bool m_driverReleased;
  };
}
#endif /*GEAR_INTAKE_SUBSYSTEM_H*/
