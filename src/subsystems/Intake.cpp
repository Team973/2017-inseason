/*
 * Intake.cpp
 *
 *  Created on: Oct 9, 2015
 *      Author: Andrew
 */

#include <subsystems/Intake.h>
#include "RobotInfo.h"
#include "lib/TaskMgr.h"
#include "lib/WrapDash.h"

namespace frc973 {

Intake::Intake(TaskMgr *scheduler) :
	m_scheduler(scheduler)
{
	this->m_scheduler->RegisterTask("Intake", this, TASK_PERIODIC);
}

Intake::~Intake() {
	this->m_scheduler->UnregisterTask(this);
}

void Intake::TaskPeriodic(RobotMode mode) {
}

}
