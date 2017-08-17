#include "GreyTalon.h"
#include "CANTalon.h"

namespace frc973{
  GreyTalon::GreyTalon(double canId, double controlPeriod, TaskMgr *scheduler):
    super(canId, controlPeriod),
    m_scheduler(scheduler),
    m_limitingMode(LimitingMode::PeakCurrentMode),
    m_peakCurrent(0.0),
    m_outputCurrent(0.0),
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
    SetLegacyMode(currentLim);
  }

  void GreyTalon::ConfigureFoldbackCurrentLimit(double peakCurrent, uint32_t maxPeakCurrentDuration, double foldbackCurrent, uint32_t maxFoldbackCurrentDuration){
    m_peakCurrent = peakCurrent;
    m_maxPeakCurrentDuration = maxPeakCurrentDuration;
    m_foldbackCurrent = foldbackCurrent;
    m_maxFoldbackCurrentDuration = maxFoldbackCurrentDuration;
    m_limitingMode = LimitingMode::PeakCurrentMode;
    SetPeakCurrentMode();
  }

  void GreyTalon::SetLegacyMode(double currentLim){
    m_limitingMode = LimitingMode::LegacyMode;
    this->CANTalon::SetCurrentLimit(currentLim);
  }

  void GreyTalon::SetPeakCurrentMode(){
    m_limitingMode = LimitingMode::PeakCurrentMode;
    m_currentDuration = -1;
    this->CANTalon::SetCurrentLimit(m_peakCurrent);
  }

  void GreyTalon::SetFoldbackMode(){
    m_currentDuration = GetMsecTime();
    m_limitingMode = LimitingMode::FoldbackCurrentMode;
    this->CANTalon::SetCurrentLimit(m_foldbackCurrent);
  }

  double GreyTalon::GetFilteredOutputCurrent(){
    return m_outputCurrent;
  }

  void GreyTalon::TaskPrePeriodic(RobotMode mode){
    m_outputCurrent = m_currentFilter->Update(this->GetOutputCurrent());
    switch (m_limitingMode) {
      case LegacyMode:
       break;
      case PeakCurrentMode:
        if (m_outputCurrent <= m_peakCurrent) {
          m_currentDuration = -1;
        }
        if (m_outputCurrent > m_peakCurrent){
          if (m_currentDuration == -1 ){
            m_currentDuration = GetMsecTime();
          }

          if (GetMsecTime() - m_currentDuration > m_maxPeakCurrentDuration){
              SetFoldbackMode();
          }
        }
       break;
       case FoldbackCurrentMode:
         if (GetMsecTime() - m_currentDuration > m_maxFoldbackCurrentDuration) {
           SetPeakCurrentMode();
         }
         break;
    }
  }
}
