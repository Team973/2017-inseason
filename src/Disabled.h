#include "lib/JoystickHelper.h"

namespace frc973 {

void Robot::DisabledStart(void) {
    fprintf(stderr, "***disable start\n");

    m_shooter->SetFlywheelStop();
	m_intake->SetIntakeMode(Intake::IntakeMode::off);
	fprintf(stderr, "disable start end \n");

	if (m_goBack == true && m_ballSnatch == true){
		DBStringPrintf(DBStringPos::DB_LINE1, "GoBack, Snatch");
	}
	else if (m_goBack == true && m_ballSnatch == false){
		DBStringPrintf(DBStringPos::DB_LINE1, "GoBack, NOSnatch");
	}
	else if (m_goBack == false && m_ballSnatch == true){
		DBStringPrintf(DBStringPos::DB_LINE1, "NOreturn, Snatch");
	}
	else if (m_goBack == false && m_ballSnatch == false){
		DBStringPrintf(DBStringPos::DB_LINE1, "NOreturn, NOsnatch");
	}
}

void Robot::DisabledStop(void) {

}

void Robot::DisabledContinuous(void) {
	/*
	printf("distance: %lf, speed: %lf\n", m_drive->GetDist(),
			m_drive->GetRate());
	printf("angle: %lf, angular rate: %lf\n", m_drive->GetAngle(),
			m_drive->GetAngularRate());
			*/
}

void Robot::HandleDisabledButton(uint32_t port, uint32_t button,
		bool pressedP){
	m_buttonPresses->LogPrintf("Button event port %d button %d pressed %d", port, button, pressedP);
	if (port == DRIVER_JOYSTICK_PORT) {
		switch (button) {
		case DualAction::BtnA:
			if (pressedP) {
				m_selectedRoutine = AutoRoutine::Portcullis;
				DBStringPrintf(DBStringPos::DB_LINE7, "Portcullis Auto");
			}
			break;
		case DualAction::BtnB:
			if (pressedP) {
				m_selectedRoutine = AutoRoutine::ChevaldeFrise;
				DBStringPrintf(DBStringPos::DB_LINE7, "ChevaldeFrise Auto");
			}
			break;
		case DualAction::BtnX:
			if (pressedP) {
				m_selectedRoutine = AutoRoutine::Drawbridge;
				DBStringPrintf(DBStringPos::DB_LINE7, "Drawbridge Auto");
			}
			break;
		case DualAction::BtnY:
			if (pressedP) {
				m_selectedRoutine = AutoRoutine::Go;
				DBStringPrintf(DBStringPos::DB_LINE7, "Forward Auto");
			}
			break;
		case DualAction::RightBumper:
			if (pressedP) {
				m_selectedRoutine = AutoRoutine::NoAuto;
				DBStringPrintf(DBStringPos::DB_LINE7, "No Auto (Just sit)");
			}
			break;
		case DualAction::DPadUpVirtBtn:
			if (pressedP) {
				m_selectedDirection = AutoStartPosition::Pos2;
				DBStringPrintf(DBStringPos::DB_LINE9, "Position 2");
			}
			break;
		case DualAction::DPadRightVirtBtn:
			if (pressedP) {
				m_selectedDirection = AutoStartPosition::Pos3;
				DBStringPrintf(DBStringPos::DB_LINE9, "Position 3");
			}
			break;
		case DualAction::DPadDownVirtBtn:
			if (pressedP) {
				m_selectedDirection = AutoStartPosition::Pos4;
				DBStringPrintf(DBStringPos::DB_LINE9, "Position 4");
			}
			break;
		case DualAction::DPadLeftVirtBtn:
			if (pressedP) {
				m_selectedDirection = AutoStartPosition::Pos5;
				DBStringPrintf(DBStringPos::DB_LINE9, "Position 5");
			}
			break;
		case DualAction::LeftBumper:
			if (pressedP) {
				m_selectedDirection = AutoStartPosition::NoVision;
				DBStringPrintf(DBStringPos::DB_LINE9, "No Vision (still drive tho)");
			}
			break;
		case DualAction::LeftTrigger:
			if (pressedP) {
				m_goBack = true;
			}
			break;
		case DualAction::RightTrigger:
			if (pressedP) {
				m_goBack = false;
			}
			break;
		case DualAction::Start:
			if (pressedP) {
				m_ballSnatch = true;
			}
			break;
		case DualAction::Back:
			if (pressedP) {
				m_ballSnatch = false;
			}
			break;
		}

		if (m_goBack == true && m_ballSnatch == true){
			DBStringPrintf(DBStringPos::DB_LINE1, "GoBack, Snatch");
		}
		else if (m_goBack == true && m_ballSnatch == false){
			DBStringPrintf(DBStringPos::DB_LINE1, "GoBack, NOSnatch");
		}
		else if (m_goBack == false && m_ballSnatch == true){
			DBStringPrintf(DBStringPos::DB_LINE1, "NOreturn, Snatch");
		}
		else if (m_goBack == false && m_ballSnatch == false){
			DBStringPrintf(DBStringPos::DB_LINE1, "NOreturn, NOsnatch");
		}
	}

}

}
