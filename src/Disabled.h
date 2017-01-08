#include "lib/JoystickHelper.h"

using namespace frc;

namespace frc973 {

void Robot::DisabledStart(void) {
    fprintf(stderr, "***disable start\n");
}

void Robot::DisabledStop(void) {
}

void Robot::DisabledContinuous(void) {
}

void Robot::HandleDisabledButton(uint32_t port, uint32_t button,
		bool pressedP){
	m_buttonPresses->LogPrintf("Button event port %d button %d pressed %d", port, button, pressedP);
	if (port == DRIVER_JOYSTICK_PORT) {
		switch (button) {
		case DualAction::BtnA:
			if (pressedP) {
			}
			break;
		case DualAction::BtnB:
			if (pressedP) {
			}
			break;
		case DualAction::BtnX:
			if (pressedP) {
			}
			break;
		case DualAction::BtnY:
			if (pressedP) {
			}
			break;
		case DualAction::RightBumper:
			if (pressedP) {
			}
			break;
		case DualAction::DPadUpVirtBtn:
			if (pressedP) {
			}
			break;
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
