#include "lib/JoystickHelper.h"
#include "lib/WrapDash.h"

namespace frc973 {

void Robot::TeleopStart(void) {
	m_teleopTimeSec = GetSecTime();
	m_drive->ArcadeDrive(0.0, 0.0);
}

void Robot::TeleopStop(void) {
}

void Robot::TeleopContinuous(void) {
	double y = m_driverJoystick->GetRawAxis(DualAction::LeftYAxis);
	double x = -m_driverJoystick->GetRawAxis(DualAction::RightXAxis);

    if (m_driverJoystick->GetRawButton(DualAction::LeftBumper)) {
        y *= 0.4;
        x *= 0.4;
    }

    m_drive->ArcadeDrive(y, -x);
}

void Robot::HandleTeleopButton(uint32_t port, uint32_t button,
		bool pressedP) {
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
		case DualAction::LeftBumper:
			if (pressedP) {
			}
			break;
		case DualAction::LeftTrigger:
			if (pressedP) {
			}
			break;
		case DualAction::RightBumper:
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
	else if (port == OPERATOR_JOYSTICK_PORT) {
		switch (button) {
		case DualAction::BtnY:
			if (pressedP) {
			}
			break;
		case DualAction::BtnA:
			if (pressedP) {
			}
			break;
		case DualAction::BtnX:
			if (pressedP) {
			}
			break;
		case DualAction::BtnB:
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
		case DualAction::RightBumper:
			if (pressedP) {
			}
			else {
			}
			break;
		case DualAction::RightTrigger:
			if (pressedP) {
			}
			break;
		case DualAction::DPadUpVirtBtn:
			if (pressedP) {
			}
			break;
		case DualAction::DPadDownVirtBtn:
			if (pressedP) {
			}
			break;
		case DualAction::DPadLeftVirtBtn:
			break;
		case DualAction::DPadRightVirtBtn:
			break;
		case DualAction::Back:
			break;
		case DualAction::Start:
			break;
		}
	}
}

}
