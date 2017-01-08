/*
 * Hanger.cpp
 *
 *  Created on: Apr 17, 2016
 *      Author: Andrew
 */

#include <subsystems/Hanger.h>
#include "subsystems/Drive.h"
#include "subsystems/Shooter.h"
#include "lib/WrapDash.h"

#include "WPILib.h"
#include "RobotInfo.h"

namespace frc973 {

Hanger::Hanger(TaskMgr *scheduler)
		 : CoopTask()
		 , m_scheduler(scheduler)
{
	m_scheduler->RegisterTask("Hanger", this, TASK_PERIODIC);
}

Hanger::~Hanger() {
	m_scheduler->UnregisterTask(this);
}

void Hanger::TaskPeriodic(RobotMode mode) {
}

} /* namespace frc973 */
