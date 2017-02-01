#include "lib/JoystickHelper.h"
#include "lib/WrapDash.h"

using namespace frc;

namespace frc973 {

static bool turretManualControl = false;

void Robot::TeleopStart(void) {
	m_teleopTimeSec = GetSecTime();
	m_drive->ArcadeDrive(0.0, 0.0);
}

void Robot::TeleopStop(void) {
}

void Robot::TeleopContinuous(void) {
	double y = m_driverJoystick->GetRawAxis(DualAction::LeftYAxis);
	double x = -m_driverJoystick->GetRawAxis(DualAction::RightXAxis);

	double turretControlPos = m_operatorJoystick->GetRawAxis(DualAction::RightXAxis);
	DBStringPrintf(DB_LINE1, "%f joystick", turretControlPos);


	if (m_driverJoystick->GetRawButton(DualAction::LeftBumper)) {
      y *= 0.4;
      x *= 0.4;
  }

  m_drive->ArcadeDrive(y, x);
}

void Robot::HandleTeleopButton(uint32_t port, uint32_t button,
		bool pressedP) {
	if (port == DRIVER_JOYSTICK_PORT) {
		switch (button) {
		case DualAction::BtnA:
			if (pressedP) {
				DBStringPrintf(DB_LINE9, "BtnA");
				m_shooter->SetFlywheelPow(1.0);
			}
			break;
		case DualAction::BtnB:
			if (pressedP) {
				DBStringPrintf(DB_LINE9, "BtnB");
				m_shooter->SetFlywheelPow(0.0);
			}
			break;
		case DualAction::BtnX:
			if (pressedP) {
				DBStringPrintf(DB_LINE9, "BtnX");
				m_ballIntake->BallIntakeStart();
			}
			break;
		case DualAction::BtnY:
			if (pressedP) {
				DBStringPrintf(DB_LINE9, "BtnY");
				m_ballIntake->BallIntakeStop();
			}
			break;
		case DualAction::LeftBumper:
			if (pressedP) {
				m_gearIntake->SetIndexerMode(GearIntake::Indexer::indexing);
			}
			break;
		case DualAction::LeftTrigger:
			if (pressedP) {
				m_gearIntake->SetIndexerMode(GearIntake::Indexer::holding);
			}
			break;
		case DualAction::RightBumper:
			if (pressedP) {
				m_gearIntake->SetIndexerMode(GearIntake::Indexer::intaking);
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
				m_gearIntake->SetGearPos(GearIntake::GearPosition::up);
			}
			break;
		case DualAction::BtnA:
			if (pressedP) {
				m_gearIntake->SetGearPos(GearIntake::GearPosition::down);
			}
			break;
		case DualAction::BtnX:
			if (pressedP) {
				m_gearIntake->SetGearIntakeState(GearIntake::GearIntakeState::released);
			}
			break;
		case DualAction::BtnB:
			if (pressedP) {
				m_gearIntake->SetGearIntakeState(GearIntake::GearIntakeState::grabbed);
			}
			break;
		case DualAction::LeftBumper:
			if (pressedP) {
				m_gearIntake->SetGearIntakeState(GearIntake::GearIntakeState::floating);
			}
			break;
		case DualAction::LeftTrigger:
			if (pressedP) {
				m_hanger->SetHangerState(Hanger::HangerState::preHang);
			}
			break;
		case DualAction::RightBumper:
			if (pressedP) {
				m_hanger->SetHangerState(Hanger::HangerState::autoHang);
			}
			else {
			}
			break;
		case DualAction::RightTrigger:
			if (pressedP) {
				m_hanger->SetHangerState(Hanger::HangerState::armed);
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
