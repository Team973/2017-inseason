/*
 *  BoilerPixyVisionDriveController.h
 *  Created: 17 February 2017
 */

 #pragma once

 #include "lib/DriveBase.h"
 #include "subsystems/BoilerPixy.h"
 #include "stdio.h"
 #include "lib/util/Util.h"

 namespace frc973{

   class BoilerPixy;
   class PID;

   class BoilerPixyVisionDriveController : public DriveController{
    public:
      BoilerPixyVisionDriveController(BoilerPixy *boilerPixy);
      virtual ~BoilerPixyVisionDriveController();

      void Start()  override{
        //m_boilerPixy->Enable();
        m_needSetControlMode = true;
        m_lightEnableTimeMs = GetMsecTime();
        m_onTarget = false;
      }

      void CalcDriveOutput(DriveStateProvider *state,
    			DriveControlSignalReceiver *out) override;

      bool OnTarget() override{
        return m_onTarget;
      };

      void SetJoystickTerm(double throttle, double turn) {
          m_joyThrottle = throttle * 0.3;
          m_joyTurn = turn * 0.3;
      }

    private:
      bool m_needSetControlMode = true;
      bool m_onTarget;
      double m_leftSetpoint;
      double m_rightSetpoint;
      uint64_t m_lightEnableTimeMs;

      double m_joyThrottle, m_joyTurn;

      BoilerPixy *m_boilerPixy;
      PID *m_pid;
   };
 }
