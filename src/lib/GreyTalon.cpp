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
    m_currentDuration(0),
    m_currentFilter(new MovingAverageFilter(0.9))
  {
    this->m_scheduler->RegisterTask("GreyTalon", this, TASK_PERIODIC);
    EnableCurrentLimit(true);
  }

  GreyTalon::~GreyTalon(){
    m_scheduler->UnregisterTask(this);
  }

  void GreyTalon::SetCurrentLimit(double currentLim){
    SetLegacyMode();
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
    SetPeakCurrentMode();
    this->SetCurrentLimit(m_peakCurrent);
  }

  void GreyTalon::SetLegacyMode(){
    m_limitingMode = LimitingMode::LegacyMode;
  }

  void GreyTalon::SetPeakCurrentMode(){
    m_limitingMode = LimitingMode::PeakCurrentMode;
  }

  void GreyTalon::SetFoldbackMode(){
    m_limitingMode = LimitingMode::FoldbackCurrentMode;
    this->SetCurrentLimit(m_foldbackCurrent);
  }

  void GreyTalon::TaskPrePeriodic(RobotMode mode){
    switch (m_limitingMode) {
      case LegacyMode:
       break;
      case PeakCurrentMode:
        if (m_currentFilter->Update(this->GetOutputCurrent()) <= m_peakCurrent) {
          m_currentDuration = -1;
        }
        if (m_currentFilter->Update(this->GetOutputCurrent()) > m_peakCurrent){
          m_currentDuration = -1;
          if (m_currentDuration == -1 ){
            m_currentDuration = GetMsecTime();
          }

          if (GetMsecTime() - m_currentDuration > m_maxPeakCurrentDuration){
              SetFoldbackMode();
          }
        }
       break;
       case FoldbackCurrentMode:
         m_currentDuration = GetMsecTime();
         if (GetMsecTime() - m_currentDuration > m_maxFoldbackCurrentDuration) {
           SetPeakCurrentMode();
         }
         break;
       default:
         break;
    }
  }
}
