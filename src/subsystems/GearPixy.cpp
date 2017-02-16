#include "GearPixy.h"
#include "RobotInfo.h"

namespace frc973{
  GearPixy::GearPixy(TaskMgr *scheduler) :
    m_scheduler(scheduler),
    m_pixyOffset(new AnalogInput(GEAR_PIXY_CAM_ANALOG)),
    m_seesTarget(new DigitalInput(GEAR_PIXY_CAM_DIGITAL))
  {

  }

  GearPixy::~GearPixy(){
    m_scheduler->UnregisterTask(this);
  }

  double GearPixy::GetOffset(){
    return m_pixyOffset->GetOffset();
  }

  bool GearPixy::GetSeesTarget(){
    return m_seesTarget->Get();
  }

  void GearPixy::TaskPeriodic(RobotMode mode){

  }
}
