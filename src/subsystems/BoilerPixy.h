#pragma once

#include "WPILib.h"
#include "lib/CoopTask.h"
#include "lib/TaskMgr.h"
#include "lib/WrapDash.h"
#include "lib/filters/MovingAverageFilter.h"
#include "lib/InterpLookupTable.h"

using namespace frc;

namespace frc973{
  class TaskMgr;
  class Lights;

  class BoilerPixy : public CoopTask{
    public:
      BoilerPixy(TaskMgr *scheduler, Lights *lights);
      virtual ~BoilerPixy();

      /**
       * Turn the flashlight on or off
       */
      void Enable();
      void Disable();

      double GetXOffset();
      double GetHeight();
      double GetXDistance();
      double GetShooterRPM();
      bool GetSeesTargetX();
      bool GetSeesTargetY();

      void TaskPeriodic(RobotMode mode) override;
    private:
      TaskMgr *m_scheduler;

      AnalogInput *m_pixyXOffset;
      AnalogInput *m_pixyYOffset;
      DigitalInput *m_seesTargetX;
      DigitalInput *m_seesTargetY;
      MovingAverageFilter *m_pixyXFilter;
      MovingAverageFilter *m_pixyYFilter;

      Lights *m_lights;
      InterpLookupTable *m_interpTable;
      InterpLookupTable *m_rpmInterpTable;
  };
}
