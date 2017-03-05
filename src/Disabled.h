#include "lib/JoystickHelper.h"

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

  if (m_autoRoutine == AutonomousRoutine::GearLeftPeg){
    DBStringPrintf(DBStringPos::DB_LINE0,
            "%c Gear to LeftPeg Auto",
            (m_alliance == Alliance::Red) ? 'R' : 'B');
  }
  else if (m_autoRoutine == AutonomousRoutine::GearMiddlePeg){
    DBStringPrintf(DBStringPos::DB_LINE0,
            "%c Gear to Middle Peg Auto",
            (m_alliance == Alliance::Red) ? 'R' : 'B');
  }
  else if (m_autoRoutine == AutonomousRoutine::GearRightPeg){
    DBStringPrintf(DBStringPos::DB_LINE0,
      "%c Gear to Right Peg Auto",
      (m_alliance == Alliance::Red) ? 'R' : 'B');
  }
  else if (m_autoRoutine == AutonomousRoutine::FuelBallToBoiler){
    DBStringPrintf(DBStringPos::DB_LINE0,
                      "%c FuelBallToBoiler Auto",
                      (m_alliance == Alliance::Red) ? 'R' : 'B');
  }
  else if (m_autoRoutine == AutonomousRoutine::ShootFuelThenHopper){
    DBStringPrintf(DBStringPos::DB_LINE0,
                      "%c ShootFuel, GoToHopper, ShootFuel Auto",
                      (m_alliance == Alliance::Red) ? 'R' : 'B');
  }
  else if (m_autoRoutine == AutonomousRoutine::HopperThenShootFuel){
    DBStringPrintf(DBStringPos::DB_LINE0,
                      "%c HopperThenShoot Auto",
                      (m_alliance == Alliance::Red) ? 'R' : 'B');
  }
  else if (m_autoRoutine == AutonomousRoutine::KpaGearAuto){
    DBStringPrintf(DBStringPos::DB_LINE0,
                      "%c KpaGear Auto",
                      (m_alliance == Alliance::Red) ? 'R' : 'B');
  }
  else if (m_autoRoutine == AutonomousRoutine::AimedAtBoilerAuto){
    DBStringPrintf(DBStringPos::DB_LINE0,
                      "%c AimedAtBoiler Auto",
                      (m_alliance == Alliance::Red) ? 'R' : 'B');
  }
  else if (m_autoRoutine == AutonomousRoutine::NoAuto){
    DBStringPrintf(DBStringPos::DB_LINE0,
                      "%c No Auto",
                      (m_alliance == Alliance::Red) ? 'R' : 'B');
  }
//    fprintf(stderr, "***disabled continuous\n");
}

void Robot::HandleDisabledButton(uint32_t port, uint32_t button,
        bool pressedP){
    m_buttonPresses->LogPrintf("Button event port %d button %d pressed %d",
            port, button, pressedP);
    switch (button) {
        case DualAction::BtnA:
            if (pressedP) {
                m_autoRoutine = AutonomousRoutine::GearLeftPeg;
            }
            break;
        case DualAction::BtnB:
            if (pressedP) {
                m_autoRoutine = AutonomousRoutine::GearMiddlePeg;
            }
            break;
        case DualAction::BtnX:
            if (pressedP) {
                m_autoRoutine = AutonomousRoutine::GearRightPeg;
            }
            break;
        case DualAction::BtnY:
            if (pressedP) {
                m_autoRoutine = AutonomousRoutine::FuelBallToBoiler;
            }
            break;
        case DualAction::RightBumper:
            if (pressedP) {
                m_autoRoutine = AutonomousRoutine::HopperThenShootFuel;
            }
            break;
        case DualAction::DPadUpVirtBtn:
            if (pressedP) {
                m_autoRoutine = AutonomousRoutine::ShootFuelThenHopper;
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
              m_autoRoutine = AutonomousRoutine::AimedAtBoilerAuto;
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
            }
            break;
        case DualAction::Start:
            if (pressedP) {
            }
            break;
        case DualAction::Back:
            if (pressedP) {
            }
            break;
    }
}

}
