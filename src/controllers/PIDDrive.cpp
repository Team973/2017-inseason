/*
 * PIDDriveController.cpp
 *
 *  Created on: Nov 5, 2015
 *      Author: Andrew
 */

#include <controllers/PIDDrive.h>
#include "lib/filters/PID.h"
#include <stdio.h>
#include "lib/WrapDash.h"

namespace frc973 {

static constexpr double DRIVE_PID_KP = 0.025;// 0.030;
static constexpr double DRIVE_PID_KI = 0.0;
static constexpr double DRIVE_PID_KD = 0.00;

static constexpr double TURN_PID_KP = 0.0105;
static constexpr double TURN_PID_KI = 0.0;
static constexpr double TURN_PID_KD = 0.00135;

static constexpr double MAX_SPEED = 150;

PIDDriveController::PIDDriveController():
	m_prevDist(0.0),
	m_prevAngle(0.0),
	m_targetDist(0.0),
	m_targetAngle(0.0),
	m_onTarget(0.0),
	m_drivePID(nullptr),
	m_turnPID(nullptr),
	m_distEnabled(true),
	m_speedCap(1.0),
    m_lastThrottle(0.0),
    m_distTolerance(DEFAULT_DIST_TOLERANCE),
    m_distRateTolerance(DEFAULT_DIST_RATE_TOLERANCE),
    m_angleTolerance(DEFAULT_ANGLE_TOLERANCE),
    m_angleRateTolerance(DEFAULT_ANGLE_RATE_TOLERANCE)
{
	m_drivePID = new PID(DRIVE_PID_KP, DRIVE_PID_KI, DRIVE_PID_KD);
	m_turnPID = new PID(TURN_PID_KP, TURN_PID_KI, TURN_PID_KD);
}

PIDDriveController::~PIDDriveController() {
    delete m_drivePID;
    delete m_turnPID;
}

void PIDDriveController::CalcDriveOutput(DriveStateProvider *state,
		DriveControlSignalReceiver *out) {
	if(m_needSetControlMode == true){
		out->SetDriveControlMode(CANSpeedController::ControlMode::kSpeed);
		m_needSetControlMode = false;
	}

	m_prevDist = state->GetDist();
	m_prevAngle = state->GetAngle();

	double throttle;
	double turn = Util::bound(m_turnPID->CalcOutput(m_prevAngle), -0.5, 0.5);

	if (m_distEnabled){
		throttle = m_drivePID->CalcOutput(m_prevDist);
	}
	else {
		throttle = 0.0;
	}

    if (throttle > m_lastThrottle + 0.09) {
        throttle = m_lastThrottle + 0.09;
    }
    m_lastThrottle = throttle;

	DBStringPrintf(DBStringPos::DB_LINE3, "p %2.2lf t %2.2lf",
            MAX_SPEED * m_speedCap * throttle,
            MAX_SPEED * m_speedCap * turn);

	printf("dist target %lf, dist curr %lf, dist error: %lf \n",
			m_targetDist, m_prevDist, m_targetDist - m_prevDist);
	printf("angle target %lf, angle curr %lf, turn error %lf\n",
			m_targetAngle, m_prevAngle, m_targetAngle - m_prevAngle);
	printf("throttle %lf  turn %lf\n",
			throttle, turn);

	DBStringPrintf(DBStringPos::DB_LINE6, "err d %.3lf a %.3lf",
            m_prevDist - m_targetDist,
            m_prevAngle - m_targetAngle);

	out->SetDriveOutput(MAX_SPEED * m_speedCap * (throttle - turn),
                        MAX_SPEED * m_speedCap * (throttle + turn));

    if (m_quickExit == false) {
        if ((m_distEnabled == false ||
                    (Util::abs(m_targetDist - m_prevDist) < m_distTolerance &&
                     Util::abs(state->GetRate()) < m_distRateTolerance)) &&
                Util::abs(m_targetAngle - m_prevAngle) < m_angleTolerance &&
                Util::abs(state->GetAngularRate()) < m_angleRateTolerance) {
            m_onTarget = true;
        }
        else {
            m_onTarget = false;
        }
    }
    else {
        if (Util::abs(m_targetDist - m_prevDist) < m_distTolerance) {
            m_onTarget = true;
        }
        else {
            m_onTarget = false;
        }
    }
}

/*
 * dist and angle are relative to current position
 */
void PIDDriveController::SetTarget(double dist, double angle,
		DriveBase::RelativeTo relativity, DriveStateProvider *state) {
	m_drivePID->Reset();
	m_turnPID->Reset();

	switch(relativity) {
	case DriveBase::RelativeTo::Absolute:
		m_targetDist = dist;
		m_targetAngle = angle;
		break;
	case DriveBase::RelativeTo::Now:
		m_targetDist = state->GetDist() + dist;
		m_targetAngle = state->GetAngle() + angle;
		break;
	case DriveBase::RelativeTo::SetPoint:
		m_targetDist = m_targetDist + dist;
		m_targetAngle = m_targetAngle + angle;
		break;
	}

	m_drivePID->SetTarget(m_targetDist);
	m_turnPID->SetTarget(m_targetAngle);

    m_distTolerance = DEFAULT_DIST_TOLERANCE;
    m_distRateTolerance = DEFAULT_DIST_RATE_TOLERANCE;
    m_angleTolerance = DEFAULT_ANGLE_TOLERANCE;
    m_angleRateTolerance = DEFAULT_ANGLE_RATE_TOLERANCE;
    m_quickExit = false;
}

}
