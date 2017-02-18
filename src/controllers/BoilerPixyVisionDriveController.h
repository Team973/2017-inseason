/*
 *  BoilerPixyVisionDriveController.h
 *  Created: 17 February 2017
 */

 #pragma once

 #include "lib/DriveBase.h"
 #include "subsystems/BoilerPixy.h"
 #include "stdio.h"

 namespace frc973{

   class BoilerPixy;

   class BoilerPixyVisionDriveController : public DriveController{
    public:
      BoilerPixyVisionDriveController(BoilerPixy *boilerPixy);
      virtual ~BoilerPixyVisionDriveController();

      void Start()  override{
        printf("Starting BoiulerPixyVisionController %p\n", m_boilerPixy);
        m_boilerPixy->Enable();
        m_needSetControlMode = true;
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

      BoilerPixy *m_boilerPixy;
   };
 }
