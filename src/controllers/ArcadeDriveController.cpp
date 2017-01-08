/*
 * ArcadeDrive.cpp
 *
 *  Created on: Oct 30, 2015
 *      Author: Andrew
 */

#include "controllers/ArcadeDriveController.h"
#include "lib/util/Util.h"
#include <stdio.h>
#include "lib/WrapDash.h"

namespace frc973 {

ArcadeDriveController::ArcadeDriveController():
	m_leftOutput(0.0),
	m_rightOutput(0.0)
{

}

ArcadeDriveController::~ArcadeDriveController() {

}

void ArcadeDriveController::CalcDriveOutput(DriveStateProvider *state,
		DriveControlSignalReceiver *out) {
	out->SetDriveOutput(m_leftOutput, m_rightOutput);
	DBStringPrintf(DBStringPos::DB_LINE4,
				"arcade l=%1.2lf r=%1.2lf", m_leftOutput, m_rightOutput);
	//printf("arcade l=%1.2lf r=%1.2lf\n", m_leftOutput, m_rightOutput);
}

void ArcadeDriveController::SetJoysticks(double throttle, double turn) {
	throttle = Util::bound(throttle, -1.0, 1.0);
	turn = Util::bound(turn, -1.0, 1.0);


	if (throttle < 0.0) {
		if (turn > 0.0) {
			m_leftOutput = (-throttle - turn);
			m_rightOutput = Util::max(-throttle, turn);
		}
		else {
			m_leftOutput = Util::max(-throttle, -turn);
			m_rightOutput = -throttle + turn;
		}
	}
	else {
		if (turn > 0.0) {
			m_leftOutput = -Util::max(throttle, turn);
			m_rightOutput = -throttle + turn;
		}
		else {
			m_leftOutput = -throttle - turn;
			m_rightOutput = -Util::max(throttle, -turn);
		}
	}

	m_leftOutput *= -1.0;
	m_rightOutput *= -1.0;

	//m_leftOutput = throttle + turn;
	//m_rightOutput = throttle - turn;
	//printf("left %lf  right %lf\n", m_leftOutput, m_rightOutput);
}

}
