#include "lib/JoystickHelper.h"
#include "lib/WrapDash.h"

using namespace frc;

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
  m_drive->ArcadeDrive(y, x);

	double c = m_operatorJoystick->GetRawAxis(DualAction::RightXAxis);

	m_shooter->StartConveyor(c);

	double l = m_operatorJoystick->GetRawAxisWithDeadband(DualAction::LeftYAxis);
	double r = m_operatorJoystick->GetRawAxisWithDeadband(DualAction::RightYAxis);

	m_shooter->Shooter::StartAgitator(l, false);
	m_shooter->Shooter::StartAgitator(r, true);
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
				m_gearIntake->SetGearIntakeState(GearIntake::GearIntakeState::floating);
			}
			break;
		case DualAction::BtnX:
			if (pressedP) {
				m_gearIntake->SetGearIntakeState(GearIntake::GearIntakeState::grabbed);
			}
			break;
		case DualAction::BtnY:
			if (pressedP) {
				m_gearIntake->SetGearIntakeState(GearIntake::GearIntakeState::released);
			}
			break;
		case DualAction::LeftBumper:
			if (pressedP) {
				//sw lowgear
			}
			break;
		case DualAction::LeftTrigger:
			if (pressedP) {
				m_gearIntake->SetReleaseAutoEnable(true);
			}
			else{
				m_gearIntake->SetReleaseAutoEnable(false);
			}
			break;
		case DualAction::RightBumper:
			if (pressedP) {
			}
			break;
		case DualAction::RightTrigger:
			if (pressedP) {
				m_gearIntake->ReleaseGear();
			}
			break;
		case DualAction::DPadUpVirtBtn:
			if (pressedP) {
				m_gearIntake->SetGearPos(GearIntake::GearPosition::up);
				//m_hanger->SetHangerState(Hanger::HangerState::autoHang);
        }
			break;
		case DualAction::DPadDownVirtBtn:
			if (pressedP) {
			}
			break;
		case DualAction::DPadLeftVirtBtn:
			if (pressedP){
				m_gearIntake->SetIndexerMode(GearIntake::Indexer::stop);
			}
			break;
		case DualAction::DPadRightVirtBtn:
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
				m_shooter->SetFlywheelSpeed(m_speedSetpt);
			}
			break;
		case DualAction::BtnX:
			if (pressedP) {
				m_ballIntake->BallIntakeStartReverse();
				}
			else{
				m_ballIntake->BallIntakeStop();
			}
			break;
		case DualAction::BtnB:
			if (pressedP) {
				m_shooter->SetFlywheelStop();
			}
			break;
		case DualAction::LeftBumper:
			if (pressedP) {
				m_gearIntake->StartPickupSequence();
			}
			break;
		case DualAction::LeftTrigger:
			if (pressedP){
				m_shooter->StartConveyor(m_conveyorSetpt);
			}
			else{
				m_shooter->StopConveyor();
			}
			break;
		case DualAction::RightBumper:
			if (pressedP) {
				m_ballIntake->BallIntakeStart();
				}
			else{
				m_ballIntake->BallIntakeStop();
			}
			break;
		case DualAction::RightTrigger:
			if (pressedP){
				m_shooter->StartAgitator(m_flailSetpt, true);
				m_shooter->StartAgitator(m_flailSetpt, false);
			}
			else{
				m_shooter->StopAgitator();
			}
			break;
		case DualAction::DPadUpVirtBtn:
			if (pressedP) {
          }
			break;
		case DualAction::DPadDownVirtBtn:
			if (pressedP) {
				m_gearIntake->SetGearPos(GearIntake::GearPosition::down);
			}
			break;
		case DualAction::DPadLeftVirtBtn:
			if (pressedP){
			}
			break;
		case DualAction::DPadRightVirtBtn:
			if (pressedP) {

				}
			break;
		case DualAction::Back:

			break;
		case DualAction::Start:

			break;
		}
	}
	else if (port == TUNING_JOYSTICK_PORT){
		switch (button) {
			case DualAction::DPadUpVirtBtn:
				if (pressedP) {
					m_speedSetpt += 50;
				}
				break;
			case DualAction::DPadDownVirtBtn:
				if (pressedP) {
					m_speedSetpt -= 50;
				}
				break;
			case DualAction::DPadRightVirtBtn:
				if (pressedP) {
					m_flailSetpt += 0.1;
				}
				break;
			case DualAction::DPadLeftVirtBtn:
				if (pressedP) {
					m_flailSetpt -= 0.1;
				}
				break;
			case DualAction::RightTrigger:
				if (pressedP) {
					m_conveyorSetpt += 0.1;
				}
				break;
			case DualAction::RightBumper:
				if (pressedP) {
					m_conveyorSetpt -= 0.1;
				}
				break;
			case DualAction::LeftTrigger:
				if (pressedP) {
				}
				break;
			case DualAction::LeftBumper:
				if (pressedP) {
				}
				break;
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
		}
	}
}

}
