/*
 * PIDDriveController.cpp
 *
 *  Created on: Nov 5, 2015
 *      Author: Andrew
 */

#include <controllers/PIDDriveController.h>
#include "lib/filters/PID.h"
#include <stdio.h>
#include "lib/WrapDash.h"

namespace frc973 {

static constexpr double DRIVE_PID_KP = 0.05;
static constexpr double DRIVE_PID_KI = 0.0;
static constexpr double DRIVE_PID_KD = 0;

static constexpr double TURN_PID_KP = 0.10;
static constexpr double TURN_PID_KI = 0.0;
static constexpr double TURN_PID_KD = 0;


/*
static constexpr double TURN_PID_KP = 0.17;
static constexpr double TURN_PID_KI = 0.002;
static constexpr double TURN_PID_KD = 0.0015;

static constexpr double TURN_FEEDFORWARD = 0.06;
*/

PIDDriveController::PIDDriveController():
	m_prevDist(0.0),
	m_prevAngle(0.0),
	m_targetDist(0.0),
	m_targetAngle(0.0),
	m_onTarget(0.0),
	m_drivePID(nullptr),
	m_turnPID(nullptr),
	m_distEnabled(true),
	m_powerCap(1.0)
{
	m_drivePID = new PID(DRIVE_PID_KP, DRIVE_PID_KI, DRIVE_PID_KD);
	m_turnPID = new PID(TURN_PID_KP, TURN_PID_KI, TURN_PID_KD);
}

void PIDDriveController::CalcDriveOutput(DriveStateProvider *state,
		DriveControlSignalReceiver *out) {
	m_prevDist = state->GetDist();
	m_prevAngle = state->GetAngle();

	double throttle;
	double turn = Util::bound(m_turnPID->CalcOutput(m_prevAngle), -0.5, 0.5);

	/*
	double turn = m_turnPID->CalcOutput(m_prevAngle);
	turn = Util::signedIncrease(turn, TURN_FEEDFORWARD);
	turn = Util::bound(turn, -0.35, 0.35);
	*/


	if (m_distEnabled){
		throttle = -Util::bound(m_drivePID->CalcOutput(m_prevDist), -m_powerCap, m_powerCap);
	}
	else {
		throttle = 0.0;
	}

	DBStringPrintf(DBStringPos::DB_LINE9, "p %2.2lf t %2.2lf", throttle, turn);

	printf("dist target %lf, dist curr %lf, dist error: %lf \n",
			m_targetDist, m_prevDist, m_targetDist - m_prevDist);
	printf("angle target %lf, angle curr %lf, turn error %lf\n",
			m_targetAngle, m_prevAngle, m_targetAngle - m_prevAngle);
	printf("throttle %lf  turn %lf\n",
			throttle, turn);

	DBStringPrintf(DBStringPos::DB_LINE6, "error %lf", m_prevAngle - m_targetAngle);

	out->SetDriveOutput(throttle + turn, throttle - turn);

	if ((m_distEnabled == false || (Util::abs(m_targetDist - m_prevDist) < 2.0 && Util::abs(state->GetRate()) < 0.5)) &&
			Util::abs(m_targetAngle - m_prevAngle) < 2.0 && Util::abs(state->GetAngularRate()) < 1.0) {
		m_onTarget = true;
	}
	else {
		m_onTarget = false;
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
}

}
