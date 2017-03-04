#include "lib/JoystickHelper.h"

using namespace frc;

namespace frc973 {

void Robot::DisabledStart(void) {
    fprintf(stderr, "***disable start\n");
}

void Robot::DisabledStop(void) {
}

void Robot::DisabledContinuous(void) {
  if (m_autoRoutine == AutonomousRoutine::GearLeftPeg){
    DBStringPrintf(DBStringPos::DB_LINE0,
            "Gear to LeftPeg Auto");
  }
  else if (m_autoRoutine == AutonomousRoutine::GearMiddlePeg){
    DBStringPrintf(DBStringPos::DB_LINE0,
            "Gear to Middle Peg Auto");
  }
  else if (m_autoRoutine == AutonomousRoutine::GearRightPeg){
    DBStringPrintf(DBStringPos::DB_LINE0,
      "Gear to Right Peg Auto");
  }
  else if (m_autoRoutine == AutonomousRoutine::FuelBallToBoiler){
    DBStringPrintf(DBStringPos::DB_LINE0,
                      "FuelBallToBoiler Auto");
  }
  else if (m_autoRoutine == AutonomousRoutine::ShootFuelThenHopper){
    DBStringPrintf(DBStringPos::DB_LINE0,
                      "ShootFuel, GoToHopper, ShootFuel Auto");
  }
  else if (m_autoRoutine == AutonomousRoutine::HopperThenShootFuel){
    DBStringPrintf(DBStringPos::DB_LINE0,
                      "HopperThenShoot Auto");
  }
  else if (m_autoRoutine == AutonomousRoutine::KpaGearAuto){
    DBStringPrintf(DBStringPos::DB_LINE0,
                      "KpaGear Auto");
  }
  else if (m_autoRoutine == AutonomousRoutine::AimedAtBoilerAuto){
    DBStringPrintf(DBStringPos::DB_LINE0,
                      "AimedAtBoiler Auto");
  }
  else if (m_autoRoutine == AutonomousRoutine::NoAuto){
    DBStringPrintf(DBStringPos::DB_LINE0,
                      "No Auto");
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
            }
            break;
        case DualAction::LeftTrigger:
            if (pressedP) {
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
