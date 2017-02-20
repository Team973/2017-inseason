/*
 *  GearPixyVisionDriveController.h
 *  Created: 19 February 2017
 */

 #pragma once

 #include "lib/DriveBase.h"
 #include "subsystems/GearPixy.h"
 #include "stdio.h"
 #include "lib/util/Util.h"
 #include "subsystems/PixyThread.h"

 namespace frc973{

   class GearPixy;
   class PID;

   class GearPixyVisionDriveController : public DriveController{
    public:
      GearPixyVisionDriveController(GearPixy *gearPixy);
      virtual ~GearPixyVisionDriveController();

      void Start()  override{
      //  m_gearPixy->Enable();
        m_needSetControlMode = true;
        m_lightEnableTimeMs = GetMsecTime();
        m_onTarget = false;
      }

      void CalcDriveOutput(DriveStateProvider *state,
    			DriveControlSignalReceiver *out) override;

      bool OnTarget() override{
        return m_onTarget;
      };

    private:
      bool m_needSetControlMode = true;
      bool m_onTarget;
      double m_leftSetpoint;
      double m_rightSetpoint;
      uint64_t m_lightEnableTimeMs;

      GearPixy *m_gearPixy;
      PID *m_pid;
   };
 }
