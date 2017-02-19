/*
 *  Lights subsystem
 *  Created: 18 February 2017
 */

#pragma once

#include "WPILib.h"
#include "RobotInfo.h"
#include "lib/CoopTask.h"
#include "lib/TaskMgr.h"

using namespace frc;

namespace frc973{
  class Lights : public CoopTask{
    public:
      Lights(TaskMgr *scheduler);
      virtual ~Lights();

      void EnableLights();
      void DisableLights();
      void NotifyFlash(int n);

      void TaskPeriodic(RobotMode mode);
    private:
      TaskMgr *m_scheduler;
      uint32_t m_lightsTimer;

      Solenoid *m_pixyLightX;
      Solenoid *m_pixyLightY;
  };
}
