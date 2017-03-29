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

  if (m_autoRoutine == AutonomousRoutine::MadtownHopperThenShootFuel){
    DBStringPrintf(DBStringPos::DB_LINE0,
                      "%c MadtownHopper",
                      (m_alliance == Alliance::Red) ? 'R' : 'B');
    m_autoSelectLog->LogPrintf(
                      "%c MadtownHopper",
                      (m_alliance == Alliance::Red) ? 'R' : 'B');
  }
  else if (m_autoRoutine == AutonomousRoutine::HopperThenShootFuel){
    DBStringPrintf(DBStringPos::DB_LINE0,
                      "%c HopperThenShoot Auto",
                      (m_alliance == Alliance::Red) ? 'R' : 'B');
    m_autoSelectLog->LogPrintf(
                      "%c HopperThenShoot Auto",
                      (m_alliance == Alliance::Red) ? 'R' : 'B');
  }
  else if (m_autoRoutine == AutonomousRoutine::KpaGearAuto){
    DBStringPrintf(DBStringPos::DB_LINE0,
                      "%c KpaGear Auto",
                      (m_alliance == Alliance::Red) ? 'R' : 'B');
    m_autoSelectLog->LogPrintf(
                      "%c KpaGear Auto",
                      (m_alliance == Alliance::Red) ? 'R' : 'B');
  }
  else if (m_autoRoutine == AutonomousRoutine::NoAuto){
    DBStringPrintf(DBStringPos::DB_LINE0,
                      "%c No Auto",
                      (m_alliance == Alliance::Red) ? 'R' : 'B');
    m_autoSelectLog->LogPrintf(
                      "%c No Auto",
                      (m_alliance == Alliance::Red) ? 'R' : 'B');
  }
  if (m_autoRoutine == AutonomousRoutine::CitrusKpaGearAuto){
    DBStringPrintf(DBStringPos::DB_LINE0,
                      "%c CitrusKpaGear Auto",
                      (m_alliance == Alliance::Red) ? 'R' : 'B');
    m_autoSelectLog->LogPrintf(
                      "%c CitrusKpaGear Auto",
                      (m_alliance == Alliance::Red) ? 'R' : 'B');
  }
}

void Robot::HandleDisabledButton(uint32_t port, uint32_t button,
        bool pressedP){
    m_buttonPresses->LogPrintf("Button event port %d button %d pressed %d",
            port, button, pressedP);
    switch (button) {
        case DualAction::BtnA:
            if (pressedP) {
                //m_autoRoutine = AutonomousRoutine::GearLeftPeg;
            }
            break;
        case DualAction::BtnB:
            if (pressedP) {
                //m_autoRoutine = AutonomousRoutine::GearMiddlePeg;
            }
            break;
        case DualAction::BtnX:
            if (pressedP) {
                //m_autoRoutine = AutonomousRoutine::GearRightPeg;
            }
            break;
        case DualAction::BtnY:
            if (pressedP) {
              //  m_autoRoutine = AutonomousRoutine::FuelBallToBoiler;
            }
            break;
        case DualAction::RightBumper:
            if (pressedP) {
                m_autoRoutine = AutonomousRoutine::HopperThenShootFuel;
            }
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
            //  m_autoRoutine = AutonomousRoutine::AimedAtBoilerAuto;
            }
            break;
        case DualAction::LeftBumper:
            if (pressedP) {
              m_alliance = Alliance::Red;
            }
            break;
        case DualAction::LeftTrigger:
            if (pressedP) {
              m_alliance = Alliance::Blue;
            }
            break;
        case DualAction::RightTrigger:
            if (pressedP) {
              m_autoRoutine = AutonomousRoutine::CitrusKpaGearAuto;
            }
            break;
        case DualAction::Start:
            if (pressedP) {
                m_autoRoutine = AutonomousRoutine::MadtownHopperThenShootFuel;
            }
            break;
        case DualAction::Back:
            if (pressedP) {
            }
            break;
    }
}

}
