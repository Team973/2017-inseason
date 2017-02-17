#include "BoilerPixy.h"
#include "RobotInfo.h"

namespace frc973{
  BoilerPixy::BoilerPixy(TaskMgr *scheduler) :
    m_scheduler(scheduler),
    m_pixyXOffset(new AnalogInput(BOILER_PIXY_CAM_X_ANALOG)),
    m_pixyYOffset(new AnalogInput(BOILER_PIXY_CAM_Y_ANALOG)),
    m_seesTargetX(new DigitalInput(BOILER_PIXY_CAM_X_DIGITAL)),
    m_seesTargetY(new DigitalInput(BOILER_PIXY_CAM_Y_DIGITAL))
  {
    m_scheduler->RegisterTask("Boiler pixy", this, TASK_PERIODIC);
  }

  BoilerPixy::~BoilerPixy(){
    m_scheduler->UnregisterTask(this);
  }

  double BoilerPixy::GetXOffset(){
    return 1.25 * (m_pixyXOffset->GetValue() - 8.0);
  }

  double BoilerPixy::GetHeight(){
    return 1.25 * (m_pixyYOffset->GetValue() - 8.0);
  }

  bool BoilerPixy::GetSeesTargetX(){
    return m_seesTargetX->Get();
  }

  bool BoilerPixy::GetSeesTargetY(){
    return m_seesTargetY->Get();
  }

  void BoilerPixy::TaskPeriodic(RobotMode mode){
      DBStringPrintf(DB_LINE3,
              "x %d %2.1f y %d %2.1f",
              GetSeesTargetX(), GetXOffset(),
              GetSeesTargetY(), GetHeight());
  }
}
