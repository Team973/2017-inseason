#include "Lights.h"

namespace frc973{
  Lights::Lights(TaskMgr *scheduler) :
    m_scheduler(scheduler),
    m_lightsTimer(0),
    m_pixyLightX(new Solenoid(BOILER_PIXY_X_LIGHT_SOL)),
    m_pixyLightY(new Solenoid(BOILER_PIXY_Y_LIGHT_SOL))
    {
      m_scheduler->RegisterTask("Lights", this, TASK_PERIODIC);
      m_pixyLightX->Set(false);
      m_pixyLightY->Set(false);
    }

    Lights::~Lights(){
      m_scheduler->UnregisterTask(this);
    }

    void Lights::EnableLights(){
      m_pixyLightX->Set(true);
      m_pixyLightY->Set(true);
    }

    void Lights::DisableLights(){
      m_pixyLightX->Set(false);
      m_pixyLightY->Set(false);
    }

    void Lights::NotifyFlash(int n){
      for (int flashOrder = 0; flashOrder < n; flashOrder++){
        m_lightsTimer = GetMsecTime();
        this->EnableLights();
        if (m_lightsTimer - GetMsecTime() >= 250){
          m_lightsTimer = GetMsecTime();
          this->DisableLights();
          if (m_lightsTimer - GetMsecTime() >= 50){
            flashOrder++;
            }
          }
        }
    }

    void Lights::TaskPeriodic(RobotMode mode){
    }
}
