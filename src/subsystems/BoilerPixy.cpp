#include "BoilerPixy.h"
#include "RobotInfo.h"
#include "Lights.h"
#include "stdio.h"
#include "lib/InterpLookupTable.h"

namespace frc973{
    BoilerPixy::BoilerPixy(TaskMgr *scheduler, Lights *lights) :
      m_scheduler(scheduler),
      m_pixyXOffset(new AnalogInput(BOILER_PIXY_CAM_X_ANALOG)),
      m_pixyYOffset(new AnalogInput(BOILER_PIXY_CAM_Y_ANALOG)),
      m_seesTargetX(new DigitalInput(BOILER_PIXY_CAM_X_DIGITAL)),
      m_seesTargetY(new DigitalInput(BOILER_PIXY_CAM_Y_DIGITAL)),
      m_pixyFilter(new MovingAverageFilter(0.9)),
      m_lights(lights)
      //m_interpTable(new InterpLookupTable())
    {
        m_scheduler->RegisterTask("Boiler pixy", this, TASK_PERIODIC);
        m_lights->DisableLights();
    }

    BoilerPixy::~BoilerPixy(){
        m_scheduler->UnregisterTask(this);
    }

    void BoilerPixy::Enable() {
        printf("Enabling the boiler pixy light \n");
        m_lights->EnableLights();
        printf("did the boiler pixy\n");
    }

    void BoilerPixy::Disable() {
        m_lights->DisableLights();
    }

    double BoilerPixy::GetXOffset(){
        return m_pixyFilter->Update(m_pixyXOffset->GetVoltage() - 2.0);
    }

    double BoilerPixy::GetHeight(){
        return 1.25 * (m_pixyYOffset->GetVoltage() - 0.8);
    }

    bool BoilerPixy::GetSeesTargetX(){
        return m_seesTargetX->Get();
    }

    bool BoilerPixy::GetSeesTargetY(){
        return m_seesTargetY->Get();
    }

    void BoilerPixy::TaskPeriodic(RobotMode mode){
        DBStringPrintf(DB_LINE7,
              "x %d %2.1lf y %d %2.4lf",
              GetSeesTargetX(), m_pixyXOffset->GetVoltage(),
              GetSeesTargetY(), m_pixyYOffset->GetVoltage());
          /*
        DBStringPrintf(DB_LINE7,
              "x %2.1lf y %2.1lf",
              m_pixyXOffset->GetVoltage(),
              m_pixyYOffset->GetVoltage());
              */
    }
}
