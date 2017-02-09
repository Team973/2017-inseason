#include "lib/JoystickHelper.h"

using namespace frc;

namespace frc973 {

void Robot::DisabledStart(void) {
    fprintf(stderr, "***disable start\n");
}

void Robot::DisabledStop(void) {
}

void Robot::DisabledContinuous(void) {
    fprintf(stderr, "***disabled continuous\n");
}

void Robot::HandleDisabledButton(uint32_t port, uint32_t button,
		bool pressedP){
	m_buttonPresses->LogPrintf("Button event port %d button %d pressed %d", port, button, pressedP);
	if (port == DRIVER_JOYSTICK_PORT) {
		switch (button) {
		/*case DualAction::BtnA:
			if (pressedP) {
        m_autoRoutine = AutonomousRoutine::GearLeftPeg;
        DBStringPrintf(DBStringPos::DB_LINE7, "Gear to LeftPeg Auto");
			}
			break;
		case DualAction::BtnB:
			if (pressedP) {
        m_autoRoutine = AutonomousRoutine::GearMiddlePeg;
        DBStringPrintf(DBStringPos::DB_LINE7, "Gear to Middle Peg Auto");
			}
			break;
		case DualAction::BtnX:
			if (pressedP) {
        m_autoRoutine = AutonomousRoutine::GearRightPeg;
        DBStringPrintf(DBStringPos::DB_LINE7, "Gear to Right Peg Auto");
			}
			break;
		case DualAction::BtnY:
			if (pressedP) {
        m_autoRoutine = AutonomousRoutine::FuelBallToBoiler;
        DBStringPrintf(DBStringPos::DB_LINE7, "FuelBallToBoiler Auto");
			}
			break;
		case DualAction::RightBumper:
			if (pressedP) {
        m_autoRoutine = AutonomousRoutine::HopperThenShootFuel;
        DBStringPrintf(DBStringPos::DB_LINE7, "Go to HopperThenShootFuel Auto");
			}
			break;
		case DualAction::DPadUpVirtBtn:
			if (pressedP) {
        m_autoRoutine = AutonomousRoutine::ShootFuelThenHopper;
        DBStringPrintf(DBStringPos::DB_LINE7, "ShootFuel, GoToHopper, ShootFuel Auto");
			}
			break;*/
		case DualAction::DPadRightVirtBtn:
			if (pressedP) {
			}
			break;
		case DualAction::DPadDownVirtBtn:
			if (pressedP) {
			}
			break;
		case DualAction::DPadLeftVirtBtn:
			if (pressedP) {
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

}
