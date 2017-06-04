#include "GreyTalon.h"

namespace frc973{
  GreyTalon::GreyTalon(double canId, double controlPeriod, TaskMgr *scheduler):
    super(canId, controlPeriod),
    m_scheduler(scheduler),
    m_limitingMode(LimitingMode::PeakCurrentMode),
    m_foldbackEnabled(false),
    m_peakCurrent(0.0),
    m_maxPeakCurrentDuration(0),
    m_foldbackCurrent(0.0),
    m_maxFoldbackCurrentDuration(0),
    m_currentDuration(0)
  {
    this->m_scheduler->RegisterTask("GreyTalon", this, TASK_PERIODIC);
    EnableCurrentLimit(true);
  }

  GreyTalon::~GreyTalon(){
    m_scheduler->UnregisterTask(this);
  }

  void GreyTalon::SetCurrentLimit(double currentLim){
    m_limitingMode = LimitingMode::LegacyMode;
    m_foldbackEnabled = false;
    //SetCurrentLimit method from parent class invokes currentLim
  }

  void GreyTalon::ConfigureFoldbackCurrentLimit(double peakCurrent, uint32_t maxPeakCurrentDuration, double foldbackCurrent, uint32_t maxFoldbackCurrentDuration){
    m_peakCurrent = peakCurrent;
    m_maxPeakCurrentDuration = maxPeakCurrentDuration;
    m_foldbackCurrent = foldbackCurrent;
    m_maxFoldbackCurrentDuration = maxFoldbackCurrentDuration;
    m_limitingMode = LimitingMode::PeakCurrentMode;
    m_foldbackEnabled = true;
    this->SetCurrentLimit(m_peakCurrent);
  }

  void GreyTalon::TaskPrePeriodic(RobotMode mode){
    switch (m_limitingMode) {
      case LegacyMode:
       break;
      case PeakCurrentMode:
        m_currentDuration = GetMsecTime();
        while (GetMsecTime() - m_currentDuration < m_maxPeakCurrentDuration) {
          if (this->GetOutputCurrent() > m_foldbackCurrent) {
            m_limitingMode = LimitingMode::FoldbackCurrentMode;
          }
        }
       break;
       case FoldbackCurrentMode:
         this->SetCurrentLimit(m_foldbackCurrent);
         m_currentDuration = GetMsecTime();
         if (GetMsecTime() - m_currentDuration > m_maxFoldbackCurrentDuration) {
           m_limitingMode = LimitingMode::PeakCurrentMode;
         }
         break;
       default:
         break;
    }
  }
}
