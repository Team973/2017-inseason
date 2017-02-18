#include "controllers/BoilerPixyVisionDriveController.h"
#include "lib/WrapDash.h"
#include "stdio.h"

namespace frc973{

  static constexpr double VISION_DRIVE_MULTIPLIER = 100.0;

  BoilerPixyVisionDriveController::BoilerPixyVisionDriveController(BoilerPixy *boilerPixy) :
    m_onTarget(false),
    m_leftSetpoint(0.0),
    m_rightSetpoint(0.0)
    {
      m_boilerPixy = boilerPixy;
    }

  BoilerPixyVisionDriveController::~BoilerPixyVisionDriveController(){
  }

  void BoilerPixyVisionDriveController::CalcDriveOutput(DriveStateProvider *state,
      DriveControlSignalReceiver *out){
    if (m_needSetControlMode){
      out->SetDriveControlMode(CANSpeedController::ControlMode::kSpeed);
      m_needSetControlMode = false;
    }

    if (m_boilerPixy->GetSeesTargetX() == false){
      m_leftSetpoint = 0.0;
      m_rightSetpoint = 0.0;
    }
    else{
      m_leftSetpoint = -VISION_DRIVE_MULTIPLIER * m_boilerPixy->GetXOffset();
      m_rightSetpoint = VISION_DRIVE_MULTIPLIER * m_boilerPixy->GetXOffset();
    }

    out->SetDriveOutput(m_leftSetpoint, m_rightSetpoint);

    if (Util::abs(m_boilerPixy->GetXOffset()) < 1.0 &&
            Util::abs(state->GetAngularRate()) < 1.0) {
  		m_onTarget = true;
  	}
  	else {
  		m_onTarget = false;
  	}
  }
\
}
