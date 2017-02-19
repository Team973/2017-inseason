#include "Lights.h"

namespace frc973{
  Lights::Lights(TaskMgr *scheduler) :
    m_scheduler(scheduler),
    m_lightsTimer(0),
    m_flashOrder(0),
    m_lightMode(LightMode::off),
    m_pixyLight(new Solenoid(BOILER_PIXY_LIGHT_SOL))
    {
      m_scheduler->RegisterTask("Lights", this, TASK_PERIODIC);
      m_pixyLight->Set(false);
    }

    Lights::~Lights(){
      m_scheduler->UnregisterTask(this);
    }

    void Lights::EnableLights(){
      m_pixyLight->Set(true);
    }

    void Lights::DisableLights(){
      m_pixyLight->Set(false);
    }

    void Lights::NotifyFlash(int n){
      m_lightMode = LightMode::blinkingOn;
      m_flashOrder = n;
    }

    void Lights::TaskPeriodic(RobotMode mode){
      switch(m_lightMode){
        case on:
          this->EnableLights();
          break;
        case off:
          this->DisableLights();
          break;
        case blinkingOn:
          if(GetMsecTime() - m_lightsTimer >= 250){
            this->EnableLights();
            m_lightsTimer = GetMsecTime();
            m_lightMode = LightMode::blinkingOff;
          }
          break;
        case blinkingOff:
          if (GetMsecTime() - m_lightsTimer >= 250){
            this->DisableLights();
            m_lightsTimer = GetMsecTime();
            m_flashOrder--;
            if (m_flashOrder > 0) {
              m_lightMode = LightMode::blinkingOn;
            }
            else {
              this->DisableLights();
            }
          }
          break;
      }
    }
}
