#pragma once

#include "WPILib.h"
#include "lib/CoopTask.h"
#include "lib/TaskMgr.h"
#include "lib/WrapDash.h"

using namespace frc;

namespace frc973{
  class TaskMgr;

  class BoilerPixy : public CoopTask{
    public:
      BoilerPixy(TaskMgr *scheduler);
      virtual ~BoilerPixy();

      double GetXOffset();
      double GetHeight();
      bool GetSeesTargetX();
      bool GetSeesTargetY();

      void TaskPeriodic(RobotMode mode) override;
    private:
      TaskMgr *m_scheduler;

      AnalogInput *m_pixyXOffset;
      AnalogInput *m_pixyYOffset;
      DigitalInput *m_seesTargetX;
      DigitalInput *m_seesTargetY;
  };
}
