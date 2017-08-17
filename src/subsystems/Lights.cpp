#include "Lights.h"
#include "lib/WrapDash.h"

namespace frc973{
  Lights::Lights(TaskMgr *scheduler) :
    m_scheduler(scheduler),
    m_lightsTimer(0),
    m_flashOrder(0),
    m_timeFlash(0),
    m_lightMode(LightMode::off),
    m_pixyLight(new Solenoid(BOILER_PIXY_LIGHT_SOL)),
    m_flashLight(new Solenoid(FLASH_LIGHT_SOL))
    {
      m_scheduler->RegisterTask("Lights", this, TASK_PERIODIC);
      m_pixyLight->Set(false);
    }

    Lights::~Lights(){
      m_scheduler->UnregisterTask(this);
    }

    /**
     * Turns on pixy lights through PCM
     */
    void Lights::EnableLights(){
      m_pixyLight->Set(true);
      m_lightMode = on;
    }

    void Lights::DisableLights(){
      m_pixyLight->Set(false);
      m_lightMode = off;
    }

    /**
     * Allows pixy lights to blink/flash for a certain number of times
     *
     * @param n     number times to flash
     * @param time  duration between each flash
     */
    void Lights::NotifyFlash(int n, uint32_t time){
      m_lightMode = LightMode::blinkingOn;
      m_flashOrder = n;
      m_timeFlash = time;
      m_lightsTimer = GetMsecTime();
    }

    void Lights::TaskPeriodic(RobotMode mode){
      switch(m_lightMode){
        case on: //both pixy anf flashlight on
          m_pixyLight->Set(true);
          m_flashLight->Set(true);
          break;
        case off:
          m_pixyLight->Set(true);
          m_flashLight->Set(true);
          break;
        case blinkingOn: //pixy light blink no flashlight
          m_pixyLight->Set(true);
          m_flashLight->Set(false);
          if(GetMsecTime() - m_lightsTimer >= m_timeFlash){
            m_lightsTimer = GetMsecTime();
            m_lightMode = LightMode::blinkingOff;
          }
          break;
        case blinkingOff: //only flashlight
          m_pixyLight->Set(false);
          m_flashLight->Set(true);
          if (GetMsecTime() - m_lightsTimer >= m_timeFlash){
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
