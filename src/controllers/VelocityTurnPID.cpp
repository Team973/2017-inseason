/*
 * VelocityTurnPID.cpp
 *
 *  Created on: Apr 3, 2016
 *      Author: Andrew
 */

#include <controllers/VelocityTurnPID.h>
#include "lib/filters/PID.h"
#include "lib/WrapDash.h"

namespace frc973 {

static constexpr double TURN_POS_KP = 0.06;
static constexpr double TURN_POS_KI = 0.0;
static constexpr double TURN_POS_KD = 0;

static constexpr double TURN_VEL_KP = 0.12;
static constexpr double TURN_VEL_KI = 0.0;
static constexpr double TURN_VEL_KD = 0.0;

static constexpr double POWER_CAP = 0.35;

VelocityTurnPID::VelocityTurnPID()
		 : m_onTarget(false)
		 , m_prevAngleVel(0.0)
		 , m_prevAnglePos(0.0)
		 , m_targetAngleVel(0.0)
		 , m_targetAnglePos(0.0)
		 , m_velPid(new PID(TURN_VEL_KP, TURN_VEL_KI, TURN_VEL_KD, PID_SPEED_CTRL))
		 , m_posPid(new PID(TURN_POS_KP, TURN_POS_KI, TURN_POS_KD)){
	m_posPid->SetBounds(-MAX_VELOCITY, MAX_VELOCITY);
	m_velPid->SetBounds(-POWER_CAP, POWER_CAP);
}

VelocityTurnPID::~VelocityTurnPID() {
	// TODO Auto-generated destructor stub
}

void VelocityTurnPID::CalcDriveOutput(DriveStateProvider *state,
			DriveControlSignalReceiver *out) {
	m_prevAngleVel = state->GetAngularRate();
	m_prevAnglePos = state->GetAngle();

	double velSetpt = m_posPid->CalcOutput(m_prevAnglePos);
	velSetpt = Util::bound(velSetpt, -MAX_VELOCITY, MAX_VELOCITY);

	m_velPid->SetTarget(velSetpt);
	double turnPow = m_velPid->CalcOutput(m_prevAngleVel);
	turnPow = Util::signedIncrease(turnPow, velSetpt * 0.1);
	turnPow = Util::bound(turnPow, -0.35, 0.35);

	out->SetDriveOutput(turnPow, -turnPow);

	DBStringPrintf(DBStringPos::DB_LINE6, "e%2.2lf v%2.2lf p%2.2lf", m_targetAnglePos - m_prevAnglePos,
			velSetpt, turnPow);

	if (Util::abs(m_targetAnglePos - m_prevAnglePos) < 0.5 &&
			Util::abs(m_prevAngleVel) < 0.5) {
		m_onTarget = true;
	}
	else {
		m_onTarget = false;
	}
}

void VelocityTurnPID::SetTarget(double angle, DriveBase::RelativeTo relativity,
		DriveStateProvider *state) {
	//m_velPid->Reset();
	m_posPid->Reset();

	switch(relativity) {
	case DriveBase::RelativeTo::Absolute:
		m_targetAnglePos = angle;
		break;
	case DriveBase::RelativeTo::Now:
		m_targetAnglePos = state->GetAngle() + angle;
		break;
	case DriveBase::RelativeTo::SetPoint:
		m_targetAnglePos = m_targetAnglePos + angle;
		break;
	}

	m_posPid->SetTarget(m_targetAnglePos);
}

} /* namespace frc973 */
