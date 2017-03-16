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
      enum LightMode {
        on,
        off,
        blinkingOn,
        blinkingOff
      };

      Lights(TaskMgr *scheduler);
      virtual ~Lights();

      void Flashlight(bool state);

      void EnableLights();
      void DisableLights();
      void NotifyFlash(int n, uint32_t time);

      void TaskPeriodic(RobotMode mode);
    private:
      TaskMgr *m_scheduler;
      uint32_t m_lightsTimer;
      int m_flashOrder;
      uint32_t m_timeFlash;

      LightMode m_lightMode;
      Solenoid *m_pixyLight;
      Solenoid *m_flashLight;
  };
}
