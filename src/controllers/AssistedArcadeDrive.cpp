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
	m_leftOutput(0.0),
	m_rightOutput(0.0),
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

	out->SetDriveOutput(-m_leftOutput, -m_rightOutput);
	DBStringPrintf(DBStringPos::DB_LINE4,
				"arcade l=%1.2lf r=%1.2lf", m_leftOutput, m_rightOutput);
	//printf("arcade l=%1.2lf r=%1.2lf\n", m_leftOutput, m_rightOutput);
}

void AssistedArcadeDriveController::SetJoysticks(double throttle, double turn) {
	throttle = Util::bound(throttle, -1.0, 1.0);
	turn = Util::bound(turn, -1.0, 1.0);

    m_leftOutput = throttle - 0.5 * DRIVE_WIDTH * turn;
    m_rightOutput = throttle + 0.5 * DRIVE_WIDTH * turn;

    double maxSpeed = Util::max(m_leftOutput, m_rightOutput);
    if (maxSpeed > 1.0) {
        m_leftOutput = m_leftOutput * (1.0 / maxSpeed);
        m_rightOutput = m_rightOutput * (1.0 / maxSpeed);
    }

	//printf("left %lf  right %lf\n", m_leftOutput, m_rightOutput);
}

}


