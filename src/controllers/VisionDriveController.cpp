/*
 * VisionDriveController.cpp
 *
 *  Created on: Feb 25, 2016
 *      Author: Andrew
 */

#include <controllers/VisionDriveController.h>
#include "lib/filters/PID.h"
#include "lib/WrapDash.h"
#include "lib/filters/Debouncer.h"
#include "stdio.h"

namespace frc973 {

static constexpr double TURN_POS_KP = 0.08;
static constexpr double TURN_POS_KI = 0.0;
static constexpr double TURN_POS_KD = 0;

static constexpr double TURN_VEL_KP = 0.12;
static constexpr double TURN_VEL_KI = 0.0;
static constexpr double TURN_VEL_KD = 0.0;

static constexpr double POWER_CAP = 0.35;

VisionDriveController::VisionDriveController()
		 : VisionDataReceiver()
		 , m_leftOutput(0.0)
		 , m_rightOutput(0.0)
		 , m_onTarget(false)
		 , m_prevAngleVel(0.0)
		 , m_prevAnglePos(0.0)
		 , m_targetAngleVel(0.0)
		 , m_targetAnglePos(0.0)
		 , m_velPid(new PID(TURN_VEL_KP, TURN_VEL_KI, TURN_VEL_KD, PID_SPEED_CTRL))
		 , m_posPid(new PID(TURN_POS_KP, TURN_POS_KI, TURN_POS_KD))
		 , m_state(WAITING)
		 , m_visionTask(new VisionTask()) {
	printf("Vision Drive Controller Has Initialized horray!!!\n");
	m_posPid->SetBounds(-MAX_VELOCITY, MAX_VELOCITY);
	m_velPid->SetBounds(-POWER_CAP, POWER_CAP);
}

VisionDriveController::~VisionDriveController() {
	;
}

void VisionDriveController::VisionReceiveXAngle(double angle) {
	printf("Received vision angle %lf\n", angle);
	m_state = TURNING;
	m_targetAnglePos = m_prevAnglePos + angle;
	m_posPid->Reset();
	m_velPid->Reset();
	m_posPid->SetTarget(m_targetAnglePos);
}

void VisionDriveController::CalcDriveOutput(DriveStateProvider *state,
		DriveControlSignalReceiver *out) {
	m_prevAnglePos = state->GetAngle();
	m_prevAngleVel = state->GetAngularRate();

	double turn = 0.0;

	if (m_state == WAITING) {
		printf("Waiting\n");
		DBStringPrintf(DBStringPos::DB_LINE4,
				"v waiting for data\n");
	}
	else if (m_state == TURNING) {
		printf("Turning\n");

		double velSetpt = m_posPid->CalcOutput(m_prevAnglePos);
		velSetpt = Util::bound(velSetpt, -MAX_VELOCITY, MAX_VELOCITY);

		m_velPid->SetTarget(velSetpt);
		turn = m_velPid->CalcOutput(m_prevAngleVel);
		turn = Util::signedIncrease(turn, velSetpt * 0.0);
		turn = Util::bound(turn, -0.35, 0.35);

		DBStringPrintf(DBStringPos::DB_LINE4,
				"v p %1.2lf a %2.1lf\n", turn, m_targetAnglePos - m_prevAnglePos);
	}
	else if (m_state == FAILING) {
		printf("Failing\n");
		DBStringPrintf(DBStringPos::DB_LINE4,
				"v failed to grab input\n");
	}

	out->SetDriveOutput(turn, -turn);
}

} /* namespace frc973 */
