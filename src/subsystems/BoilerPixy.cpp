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
      m_pixyXFilter(new MovingAverageFilter(0.9)),
      m_pixyYFilter(new MovingAverageFilter(0.9)),
      m_lights(lights),
      m_interpTable(new InterpLookupTable()),
      m_rpmInterpTable(new InterpLookupTable())
    {
        m_scheduler->RegisterTask("Boiler pixy", this, TASK_PERIODIC);
        m_lights->DisableLights();
        /*
          x = pixy y offset
          y = horizontal distance from edge of boiler to edge of front bumper
        */
        //comp bot
        /*m_interpTable->AddPoint(2.3352,30.0);
        m_interpTable->AddPoint(2.1558,36.0);
        m_interpTable->AddPoint(2.0093,42.0);
        m_interpTable->AddPoint(1.8604,48.0);
        m_interpTable->AddPoint(1.7297,54.0);
        m_interpTable->AddPoint(1.615,60.0);
        m_interpTable->AddPoint(1.5344,66.0);
        m_interpTable->AddPoint(1.4197,72.0);
        m_interpTable->AddPoint(1.3684,78.0);*/
        //pbot
        //m_interpTable->AddPoint(3.2361,0.0);
        //m_interpTable->AddPoint(3.0408,6.0);
        //m_interpTable->AddPoint(2.8271,12.0);
      /*  m_interpTable->AddPoint(2.5635,18.0);
        m_interpTable->AddPoint(2.334,24.0);
        m_interpTable->AddPoint(2.1863,30.0);
        m_interpTable->AddPoint(2.0081,36.0);
        m_interpTable->AddPoint(1.8433,42.0);
        m_interpTable->AddPoint(1.6956,48.0);
        m_interpTable->AddPoint(1.582,54.0);
        m_interpTable->AddPoint(1.4673,60.0);*/
        m_interpTable->AddPoint(1.8962,54.0);
        m_interpTable->AddPoint(1.7609,61.0);
        m_interpTable->AddPoint(1.5979,70.0);
        m_interpTable->AddPoint(2.0238,48.0);
        m_interpTable->AddPoint(1.4514,81.0);

        /*
          x = horizontal distance from edge of boiler to edge of front bumper
          y = RPM
        */
        m_rpmInterpTable->AddPoint(91,2900);
        //m_rpmInterpTable->AddPoint(,);
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
        return m_pixyXFilter->Update(m_pixyXOffset->GetVoltage() - 2.0);
    }

    double BoilerPixy::GetHeight(){
        return m_pixyYFilter->Update(1.25 * (m_pixyYOffset->GetVoltage() - 0.8));
    }

    double BoilerPixy::GetXDistance(){
      return m_interpTable->LookupPoint(m_pixyYFilter->Update(m_pixyYOffset->GetVoltage()));
    }

    double BoilerPixy:: GetShooterRPM(){
      return m_rpmInterpTable->LookupPoint(GetXDistance());
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
        DBStringPrintf(DB_LINE3, "horizontal %2.4lf", GetXDistance());
    }
}
