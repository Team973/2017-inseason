#pragma once

#include "WPILib.h"
#include "lib/CoopTask.h"
#include "lib/TaskMgr.h"
#include "lib/WrapDash.h"
#include "PixyThread.h"

using namespace frc;

namespace frc973{
  class TaskMgr;

  class GearPixy : public CoopTask{
    public:
      GearPixy(TaskMgr *scheduler);
      virtual ~GearPixy();

      double GetOffset();
      bool GetSeesTarget();

      void TaskPeriodic(RobotMode mode) override;
    private:
      TaskMgr *m_scheduler;

      AnalogInput *m_pixyOffset;
      DigitalInput *m_seesTarget;
  };
}
