#include "controllers/BoilerPixyVisionDriveController.h"
#include "lib/WrapDash.h"
#include "stdio.h"
#include "lib/filters/PID.h"

namespace frc973{

  static constexpr double VISION_DRIVE_MULTIPLIER = 90.0;

  BoilerPixyVisionDriveController::BoilerPixyVisionDriveController(BoilerPixy *boilerPixy) :
    m_onTarget(false),
    m_leftSetpoint(0.0),
    m_rightSetpoint(0.0),
    m_lightEnableTimeMs(0),
    m_joyThrottle(0),
    m_joyTurn(0),
    m_boilerPixy(boilerPixy),
    m_pid(new PID(0.50, 0.0, 0.025))
  {
  }

  BoilerPixyVisionDriveController::~BoilerPixyVisionDriveController(){
      delete m_pid;
  }

  void BoilerPixyVisionDriveController::CalcDriveOutput(DriveStateProvider *state,
      DriveControlSignalReceiver *out){
    if (m_needSetControlMode){
      out->SetDriveControlMode(CANSpeedController::ControlMode::kSpeed);
      m_needSetControlMode = false;
    }

    double offset = m_boilerPixy->GetXOffset();

    if (m_boilerPixy->GetSeesTargetX() == false ||
        GetMsecTime() - m_lightEnableTimeMs < 100){
      m_leftSetpoint = 0.0;
      m_rightSetpoint = 0.0;
    }
    else{
      double pidOut = m_pid->CalcOutput(offset);
      m_leftSetpoint = VISION_DRIVE_MULTIPLIER * (-pidOut + m_joyThrottle - m_joyTurn);
      m_rightSetpoint = VISION_DRIVE_MULTIPLIER * (pidOut + m_joyThrottle + m_joyTurn);
    }

    out->SetDriveOutput(m_leftSetpoint, m_rightSetpoint);

    if (Util::abs(offset) < 5.0 &&
            Util::abs(state->GetAngularRate()) < 1.0) {
  		m_onTarget = true;
  	}
  	else {
  		m_onTarget = false;
  	}
  }

}
