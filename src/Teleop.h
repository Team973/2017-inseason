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
    printf("throttle  %lf, turn  %lf\n", y, x);

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
                m_gearIntake->SetGearPos(GearIntake::GearPosition::up);
			}
			break;
		case DualAction::BtnB:
			if (pressedP) {
                m_gearIntake->SetGearPos(GearIntake::GearPosition::down);
			}
			break;
		case DualAction::BtnX:
			if (pressedP) {
                m_gearIntake->SetGearIntakeState(
                        GearIntake::GearIntakeState::released);
			}
			break;
		case DualAction::BtnY:
			if (pressedP) {
                m_gearIntake->SetGearIntakeState(
                        GearIntake::GearIntakeState::grabbed);
			}
			break;
		case DualAction::LeftBumper:
			if (pressedP) {
				m_gearIntake->SetIndexerMode(GearIntake::Indexer::indexing);
			}
			break;
		case DualAction::LeftTrigger:
			if (pressedP) {
				m_gearIntake->SetGearIntakeState(GearIntake::GearIntakeState::floating);
			}
			break;
		case DualAction::RightBumper:
			if (pressedP) {
				m_gearIntake->SetIndexerMode(GearIntake::Indexer::intaking);
			}
			break;
		case DualAction::RightTrigger:
			if (pressedP) {
                m_gearIntake->SetIndexerMode(GearIntake::Indexer::stop);
			}
			break;
		case DualAction::Start:
			if (pressedP) {
				m_gearIntake->StartPickupSequence();
			}
			break;
		case DualAction::Back:
			if (pressedP) {
				m_gearIntake->ReleaseGear();
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
                m_ballIntake->BallIntakeStop();
			}
			break;
		case DualAction::LeftTrigger:
			if (pressedP) {
                m_ballIntake->BallIntakeStart();
			}
			break;
		case DualAction::RightBumper:
			if (pressedP) {
                m_shooter->StopAgitator();
			}
			break;
		case DualAction::RightTrigger:
			if (pressedP) {
			    m_shooter->StartAgitator();
            }
			break;
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
		case DualAction::DPadLeftVirtBtn:
			if (pressedP){
				m_shooter->SetFlywheelSpeed(m_speedSetpt);
			}
			break;
		case DualAction::DPadRightVirtBtn:
			break;
		case DualAction::Back:
			break;
		case DualAction::Start:
            if (pressedP) {
                m_shooter->StartConveyor();
            }
            else {
                m_shooter->StopConveyor();
            }
			break;
		}
	}
}

}
