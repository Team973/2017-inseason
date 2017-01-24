/*
 * Hanger.h
 *
 *  Created on: Apr 17, 2016
 *      Author: Andrew
 */

#pragma once

#include "lib/CoopTask.h"
#include "lib/TaskMgr.h"

using namespace frc;

namespace frc973 {

class Hanger : public CoopTask {
public:
	Hanger(TaskMgr *scheduler);
	virtual ~Hanger();
	void TaskPeriodic(RobotMode mode);
private:
	TaskMgr *m_scheduler;
};

} /* namespace frc973 */
