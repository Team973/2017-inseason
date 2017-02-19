#include "BoilerPixy.h"
#include "RobotInfo.h"
#include "stdio.h"

namespace frc973{
    BoilerPixy::BoilerPixy(TaskMgr *scheduler) :
    m_scheduler(scheduler),
    m_pixyXOffset(new AnalogInput(BOILER_PIXY_CAM_X_ANALOG)),
    m_pixyYOffset(new AnalogInput(BOILER_PIXY_CAM_Y_ANALOG)),
    m_seesTargetX(new DigitalInput(BOILER_PIXY_CAM_X_DIGITAL)),
    m_seesTargetY(new DigitalInput(BOILER_PIXY_CAM_Y_DIGITAL)),
    m_pixyLight(new Solenoid(BOILER_PIXY_LIGHT_SOL)),
    m_pixyFilter(new MovingAverageFilter(0.9))
    {
        m_scheduler->RegisterTask("Boiler pixy", this, TASK_PERIODIC);
        m_pixyLight->Set(false);
    }

    BoilerPixy::~BoilerPixy(){
        m_scheduler->UnregisterTask(this);
    }

    void BoilerPixy::Enable() {
        printf("Enabling the boiler pixy light %p\n", m_pixyLight);
        m_pixyLight->Set(true);
        printf("did the boiler pixy\n");
    }

    void BoilerPixy::Disable() {
        m_pixyLight->Set(false);
    }

    double BoilerPixy::GetXOffset(){
        return m_pixyFilter->Update(m_pixyXOffset->GetVoltage() - 1.6);
    }

    double BoilerPixy::GetHeight(){
        return 1.25 * (m_pixyYOffset->GetVoltage() - 8.0);
    }

    bool BoilerPixy::GetSeesTargetX(){
        return m_seesTargetX->Get();
    }

    bool BoilerPixy::GetSeesTargetY(){
        return m_seesTargetY->Get();
    }

    void BoilerPixy::TaskPeriodic(RobotMode mode){
        DBStringPrintf(DB_LINE7,
              "x %d %2.1lf y %d %2.1lf",
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
