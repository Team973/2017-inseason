#pragma once

#include "RobotInfo.h"
#include "WPILib.h"
#include "CANTalon.h"
#include "lib/CoopTask.h"

using namespace frc;

namespace frc973 {

  class TaskMgr;
  class PixyThread;

  class Turret: public CoopTask{
    public:
      Turret(TaskMgr *m_scheduler, LogSpreadsheet *logger, PixyThread *pixy);
      virtual ~Turret();

      enum TurretState{
        runningPow,
        runningPos,
        notRunning,
        vision
      };
      void SetTurretPosition(double position);
      double GetTurretPosition();
      void StopTurret();
      void SetTurretPower(double power);
      void SetTurretMode(TurretState turretState);
      void SetTurretAutoTarget();

      void TaskPeriodic(RobotMode mode) override;

    private:
      CANTalon *m_turretMotor;
      TaskMgr *m_scheduler;
      Solenoid *m_greenFlashlight;

      TurretState m_turretState;

      double m_turretPos;
      double m_turretPower;
      PixyThread *m_pixyThread;
  };
}
