/*
 * ArcadeDrive.cpp
 *
 *  Created on: Oct 30, 2015
 *      Author: Andrew
 */

#include "controllers/AssistedArcadeDrive.h"
#include "lib/util/Util.h"
#include <stdio.h>
#include "lib/WrapDash.h"
#include "RobotInfo.h"

namespace frc973 {

AssistedArcadeDriveController::AssistedArcadeDriveController():
	m_throttle(0.0),
	m_turn(0.0),
	m_needSetControlMode(true)
{
}

AssistedArcadeDriveController::~AssistedArcadeDriveController() {

}

void AssistedArcadeDriveController::CalcDriveOutput(DriveStateProvider *state,
		DriveControlSignalReceiver *out) {
	if(m_needSetControlMode == true){
		out->SetDriveControlMode(CANSpeedController::ControlMode::kPercentVbus);
		m_needSetControlMode = false;
	}

	double currAngRate = state->GetAngularRate();
	double leftOutput;
	double rightOutput;

	m_throttle = Util::bound(m_throttle, -1.0, 1.0);
	m_turn = Util::bound(m_turn, -1.0, 1.0);

	double setpoint = 5.0 * m_turn;
	double error = setpoint - currAngRate;
	double turnCorrection = Util::bound(error * 0.008, -0.2, 0.2);

	leftOutput = m_throttle - m_turn - turnCorrection;
	rightOutput = m_throttle + m_turn + turnCorrection;

  double maxSpeed = Util::max(Util::abs(leftOutput), Util::abs(rightOutput));
  if (maxSpeed > 1.0) {
      leftOutput = leftOutput * (1.0 / maxSpeed);
      rightOutput = rightOutput * (1.0 / maxSpeed);
	}

	out->SetDriveOutput(-leftOutput, -rightOutput);
	DBStringPrintf(DBStringPos::DB_LINE4,
				"arcade l=%1.2lf r=%1.2lf", leftOutput, rightOutput);
	//printf("arcade l=%1.2lf r=%1.2lf\n", leftOutput, rightOutput);
}

void AssistedArcadeDriveController::SetJoysticks(double throttle, double turn) {
	m_throttle = throttle;
	m_turn = turn;
}

	//printf("left %lf  right %lf\n", m_leftOutput, m_rightOutput);
}
