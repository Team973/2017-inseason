/*
 * Hanger.h
 *
 *  Created on: Apr 17, 2016
 *      Author: Andrew
 */

#ifndef SRC_SUBSYSTEMS_HANGER_H_
#define SRC_SUBSYSTEMS_HANGER_H_

#include "lib/CoopTask.h"
#include "lib/TaskMgr.h"

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

#endif /* SRC_SUBSYSTEMS_HANGER_H_ */
