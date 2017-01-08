/*
 * RampPIDDriveController.cpp
 *
 *  Created on: Nov 5, 2015
 *      Author: Andrew
 */

#include <controllers/RampedPIDDriveController.h>
#include "lib/filters/PID.h"
#include "lib/filters/RampedOutput.h"
#include <stdio.h>
#include "lib/WrapDash.h"

namespace frc973 {

static constexpr double DRIVE_PID_KP = 0.1;
static constexpr double DRIVE_PID_KI = 0.0;
static constexpr double DRIVE_PID_KD = 0.01;

/*
 * Units in inches per second
 */
static constexpr double DRIVE_RAMP_RATE = 2.0;

static constexpr double TURN_PID_KP = 0.22;
static constexpr double TURN_PID_KI = 0.001;
static constexpr double TURN_PID_KD = 0.0;

/*
 * Units in degrees per second
 */
static constexpr double TURN_RAMP_RATE = 1.0;

RampPIDDriveController::RampPIDDriveController():
	m_prevDist(0.0),
	m_prevAngle(0.0),
	m_targetDist(0.0),
	m_targetAngle(0.0),
	m_onTarget(0.0),
	m_drivePID(new PID(DRIVE_PID_KP, DRIVE_PID_KI, DRIVE_PID_KD)),
	m_turnPID(new PID(TURN_PID_KP, TURN_PID_KI, TURN_PID_KD)),
	m_driveRampFilter(new RampedOutput(DRIVE_RAMP_RATE)),
	m_turnRampFilter(new RampedOutput(TURN_RAMP_RATE)),
	m_distEnabled(true)
{
	;
}

void RampPIDDriveController::CalcDriveOutput(DriveStateProvider *state,
		DriveControlSignalReceiver *out) {
	m_prevDist = state->GetDist();
	m_prevAngle = state->GetAngle();

	m_drivePID->SetTarget(m_driveRampFilter->Update(m_targetDist));
	m_turnPID->SetTarget(m_turnRampFilter->Update(m_targetAngle));

	DBStringPrintf(DBStringPos::DB_LINE9, "angle setpt %lf",
			m_turnRampFilter->GetLast());

	double throttle = -Util::bound(m_drivePID->CalcOutput(m_prevDist), -0.5, 0.5);
	double turn = Util::bound(m_turnPID->CalcOutput(m_prevAngle), -0.5, 0.5);

	printf("dist target %lf, dist curr %lf, dist error: %lf \n",
			m_targetDist, m_prevDist, m_targetDist - m_prevDist);
	printf("angle target %lf, angle curr %lf, turn error %lf\n",
			m_targetAngle, m_prevAngle, m_targetAngle - m_prevAngle);
	printf("throttle %lf  turn %lf\n",
			throttle, turn);

	DBStringPrintf(DBStringPos::DB_LINE6, "error %lf", m_prevAngle - m_targetAngle);

	out->SetDriveOutput(throttle - turn, throttle + turn);

	if (Util::abs(m_targetDist - m_prevDist) < 2 && Util::abs(state->GetRate()) < 0.5) {
		m_onTarget = true;
	}
	else {
		m_onTarget = false;
	}
}

/*
 * dist and angle are relative to current position
 */
void RampPIDDriveController::SetTarget(double dist, double angle) {
	m_targetDist += dist;
	m_targetAngle += angle;
}

}
