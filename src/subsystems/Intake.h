/*
 * Intake.h
 *
 *  Created on: Oct 9, 2015
 *      Author: Andrew
 *
 * Intake module handles extending and retracting the the intake arm, and
 * running the roller on the end of that intake arm.
 *
 * Arm extends with the powering of one solenoid.  It retracts with the
 * unpowering of that solenoid because there's surgical tubing pulling
 * it in.
 */

#ifndef SRC_SUBSYSTEMS_INTAKE_H_
#define SRC_SUBSYSTEMS_INTAKE_H_

#include "WPILib.h"
#include "lib/CoopTask.h"

using namespace frc;

namespace frc973 {

class Intake : public CoopTask {
public:
	Intake(TaskMgr *scheduler);
	virtual ~Intake();
	void TaskPeriodic(RobotMode mode);
private:
	TaskMgr *m_scheduler;
};

}

#endif /* SRC_SUBSYSTEMS_INTAKE_H_ */
