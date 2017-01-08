/*
 * DriveBase.cpp
 *
 *  Created on: Oct 29, 2015
 *      Author: Andrew
 */

#include <lib/DriveBase.h>
#include "lib/util/Util.h"

#include "WPILib.h"

namespace frc973 {

DriveBase::DriveBase(TaskMgr *scheduler, DriveStateProvider *state,
		DriveControlSignalReceiver *outpt, DriveController *controller)
		 : m_scheduler(scheduler)
		 , m_stateProvider(state)
		 , m_driveOutput(outpt)
		 , m_controller(controller)
{
	m_scheduler->RegisterTask("DriveBase", this, TASK_POST_PERIODIC);
}

DriveBase::~DriveBase() {
	m_scheduler->UnregisterTask(this);
}


void DriveBase::TaskPostPeriodic(RobotMode mode) {
	if (m_controller != nullptr) {
		m_controller->CalcDriveOutput(m_stateProvider, m_driveOutput);
	}
}

void DriveBase::SetDriveController(DriveController *newController) {
	DriveController *oldController = m_controller;

	if (m_controller != nullptr && newController != oldController) {
		m_controller->Stop();
	}

	m_controller = newController;

	if (m_controller != nullptr && newController != oldController) {
		m_controller->Start();
	}
}

bool DriveBase::OnTarget() {
	if (m_controller) {
		return m_controller->OnTarget();
	}
	else {
		return false;
	}
}

}
