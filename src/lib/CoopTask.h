/*
 * CoopTask.h
 *
 *  Created on: Sep 5, 2015
 *      Author: Andrew
 *
 * Interface for objects that can be run by a TaskMgr.  Similar to 254's
 * Loopable interface.
 *
 * TaskMgr keeps a collection of these objects and runs them repeatedly when
 * appropriate.  In general, TaskPrePeriodic should be for tasks that read
 * sensors or calculate values that other tasks may need to read, TaskPeriodic
 * should be used for most things, and TaskPostPeriodic should be used for
 * tasks that might conflict with other Periodic tasks.  A task may register
 * any combination of hooks.
 */

#ifndef FRCLIB_COOPTASK_H_
#define FRCLIB_COOPTASK_H_

#include "util/Util.h"

namespace frc973 {

class TaskMgr;

class CoopTask {
public:
	CoopTask();
	virtual ~CoopTask();

	virtual void TaskStartMode(RobotMode mode) {}
	virtual void TaskStopMode(RobotMode mode) {}
	virtual void TaskPrePeriodic(RobotMode mode) {}
	virtual void TaskPeriodic(RobotMode mode) {}
	virtual void TaskPostPeriodic(RobotMode mode) {}
};

}

#endif /* FRCLIB_COOPTASK_H_ */
