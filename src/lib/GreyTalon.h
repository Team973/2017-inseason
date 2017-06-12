#include "stdio.h"
#include "WPILib.h"
#include "CANTalon.h"
#include "lib/util/Util.h"
#include "lib/WrapDash.h"
#include "lib/TaskMgr.h"
#include "lib/CoopTask.h"
#include "lib/filters/MovingAverageFilter.h"

using namespace frc;

namespace frc973{
  class TaskMgr;

  class GreyTalon :
      public CANTalon,
      public CoopTask{
         public:
           typedef CANTalon super;
           GreyTalon(double canId, double controlPeriod, TaskMgr *scheduler);
           virtual ~GreyTalon();

           void SetCurrentLimit(double currentLim);
           void ConfigureFoldbackCurrentLimit(double peakCurrent, uint32_t maxPeakCurrentDuration, double foldbackCurrent, uint32_t maxFoldbackCurrentDuration);
           void SetLegacyMode();
           void SetPeakCurrentMode();
           void SetFoldbackMode();
           void TaskPrePeriodic(RobotMode mode) override;

           enum LimitingMode{
             LegacyMode,
             PeakCurrentMode,
             FoldbackCurrentMode
           };
         private:
           TaskMgr       *m_scheduler;
           LimitingMode  m_limitingMode;

           bool       m_foldbackEnabled;
           double     m_peakCurrent;
           uint32_t   m_maxPeakCurrentDuration;
           double     m_foldbackCurrent;
           uint32_t   m_maxFoldbackCurrentDuration;
           uint32_t   m_currentDuration;
           MovingAverageFilter  m_currentFilter;
     };

}
