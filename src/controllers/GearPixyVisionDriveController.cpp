#include "controllers/GearPixyVisionDriveController.h"
#include "lib/WrapDash.h"
#include "stdio.h"
#include "lib/filters/PID.h"

namespace frc973{

  static constexpr double VISION_DRIVE_MULTIPLIER = 200.0;

  GearPixyVisionDriveController::GearPixyVisionDriveController(PixyThread *gearPixy) :
    m_onTarget(false),
    m_leftSetpoint(0.0),
    m_rightSetpoint(0.0),
    m_gearPixy(gearPixy),
    m_pid(new PID(8.0, 0.0, 0.0))
  {
  }

  GearPixyVisionDriveController::~GearPixyVisionDriveController(){
  }

  void GearPixyVisionDriveController::CalcDriveOutput(DriveStateProvider *state,
      DriveControlSignalReceiver *out){
    if (m_needSetControlMode){
      out->SetDriveControlMode(CANSpeedController::ControlMode::kSpeed);
      m_needSetControlMode = false;
    }

    double offset = m_gearPixy->GetOffset();

    if (m_gearPixy->GetDataFresh() == false){
      m_leftSetpoint = 0.0;
      m_rightSetpoint = 0.0;
    }
    else{
      double pidOut = m_pid->CalcOutput(offset);
      m_leftSetpoint = -VISION_DRIVE_MULTIPLIER * pidOut;
      m_rightSetpoint = VISION_DRIVE_MULTIPLIER * pidOut;
    }

    out->SetDriveOutput(m_leftSetpoint, m_rightSetpoint);

    if (Util::abs(m_gearPixy->GetOffset()) < 0.04 &&
            Util::abs(state->GetAngularRate()) < 1.0) {
  		m_onTarget = true;
  	}
  	else {
  		m_onTarget = false;
  	}
  }
\
}
