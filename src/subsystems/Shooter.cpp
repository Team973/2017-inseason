/*
 * Shooter.cpp
 *
 *  Created on: Oct 9, 2015
 *      Author: Andrew
 */

#include "RobotInfo.h"
#include "lib/util/Util.h"
#include "lib/WrapDash.h"
#include "lib/TaskMgr.h"
#include "lib/logging/LogSpreadsheet.h"
#include "Shooter.h"

namespace frc973 {

Shooter::Shooter(TaskMgr *scheduler, LogSpreadsheet *logger) :
		m_scheduler(scheduler)
{
	m_scheduler->RegisterTask("Shooter", this, TASK_PERIODIC);
}

Shooter::~Shooter() {
	m_scheduler->UnregisterTask(this);
}

void Shooter::TaskPeriodic(RobotMode mode) {
}

}
