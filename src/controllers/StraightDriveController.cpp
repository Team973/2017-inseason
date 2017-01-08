/*
 *  Created on: Oct 30, 2015
 *      Author: Andrew
 */

#include "controllers/StraightDriveController.h"
#include "lib/util/Util.h"
#include "lib/filters/PID.h"
#include <stdio.h>

namespace frc973 {

StraightDriveController::StraightDriveController():
	m_leftOutput(0.0),
	m_rightOutput(0.0),
	m_turnPid(new PID(0.0001, 0.0, 0.0)),
	m_targetHeading(0.0),
	m_throttle(0.0)
{
	;
}

StraightDriveController::~StraightDriveController() {
	;
}

void StraightDriveController::CalcDriveOutput(DriveStateProvider *state,
		DriveControlSignalReceiver *out) {
	double throttle = m_throttle;
	double turn = m_turnPid->CalcOutput(state->GetAngle());

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
	printf("left %lf  right %lf\n", m_leftOutput, m_rightOutput);

	out->SetDriveOutput(m_leftOutput, m_rightOutput);
}

void StraightDriveController::SetJoysticks(double throttle) {
	m_throttle = Util::bound(throttle, -1.0, 1.0);
}

void StraightDriveController::SetTargetHeading(double headingDegrees) {
	m_targetHeading = headingDegrees;
	m_turnPid->SetTarget(m_targetHeading);
}

}
