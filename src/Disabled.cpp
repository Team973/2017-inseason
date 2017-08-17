#include "Robot.h"
#include "subsystems/Shooter.h"
#include "subsystems/Drive.h"
#include "subsystems/GearIntake.h"
#include "lib/GreyCompressor.h"
#include "subsystems/BallIntake.h"
#include "controllers/PIDDrive.h"
#include "lib/JoystickHelper.h"
#include "lib/WrapDash.h"

using namespace frc;

namespace frc973 {

void Robot::DisabledStart(void) {
    fprintf(stderr, "***disable start\n");
}

void Robot::DisabledStop(void) {
}

void Robot::DisabledContinuous(void) {
  if(m_alliance == Alliance::Red){
      m_autoDirection = -1.0;
  }
  else if(m_alliance == Alliance::Blue){
      m_autoDirection = 1.0;
  }

  DBStringPrintf(DBStringPos::DB_LINE0,
                 "%c %s %s",
                 (m_alliance == Alliance::Red) ? 'R' : 'B',
                 (m_endMode) ? "DriveBack" : "Stay",
                 GetAutoName(m_autoRoutine));
}

void Robot::HandleDisabledButton(uint32_t port, uint32_t button,
        bool pressedP){
    m_buttonPresses->LogPrintf("Button event port %d button %d pressed %d",
            port, button, pressedP);
    switch (button) {
        case DualAction::BtnA:
            if (pressedP) {
                m_endMode = false;
            }
            break;
        case DualAction::BtnB:
            if (pressedP) {
                m_autoRoutine = AutonomousRoutine::KillerHopper;
            }
            break;
        case DualAction::BtnX:
            if (pressedP) {
                m_autoRoutine = AutonomousRoutine::MidPegKpa;
            }
            break;
        case DualAction::BtnY:
            if (pressedP) {
              m_endMode = true;
            }
            break;
        case DualAction::RightBumper:
            /*
            if (pressedP) {
                m_autoRoutine = AutonomousRoutine::HopperThenShootFuel;
            }
            */
            break;
        case DualAction::DPadUpVirtBtn:
            if (pressedP) {
              //  m_autoRoutine = AutonomousRoutine::ShootFuelThenHopper;
            }
            break;
        case DualAction::DPadRightVirtBtn:
            if (pressedP) {
              m_autoRoutine = AutonomousRoutine::NoAuto;
            }
            break;
        case DualAction::DPadDownVirtBtn:
            if (pressedP) {
              m_autoRoutine = AutonomousRoutine::KpaGearAuto;
            }
            break;
        case DualAction::DPadLeftVirtBtn:
            if (pressedP) {
                m_autoRoutine = AutonomousRoutine::CitrusHopper;
            }
            break;
        case DualAction::LeftBumper:
            if (pressedP) {
              m_alliance = Alliance::Red;
            }
            break;
        case DualAction::LeftTrigger:
            if (pressedP) {
              //m_alliance = Alliance::Blue;
            }
            break;
        case DualAction::RightTrigger:
            if (pressedP) {
              m_autoRoutine = AutonomousRoutine::CitrusKpaGearAuto;
            }
            break;
        case DualAction::Start:
            /*
            if (pressedP) {
                m_autoRoutine = AutonomousRoutine::MadtownHopperThenShootFuel;
            }
            */
            break;
        case DualAction::Back:
            /*
            if (pressedP) {
                m_austinGyro->Calibrate();
            }
            */
            break;
    }
}

}
