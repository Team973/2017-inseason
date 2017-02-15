/*
 * Hanger.h
 *
 *  Created on: Apr 17, 2016
 *      Author: Andrew
 */

#pragma once

#include "lib/CoopTask.h"
#include "lib/TaskMgr.h"
#include "CANTalon.h"

using namespace frc;

namespace frc973 {

class Hanger : public CoopTask {
public:
	enum HangerState {
		autoHang,
		preHang,
		armed
	};

	Hanger(TaskMgr *scheduler);
	virtual ~Hanger();
	void TaskPeriodic(RobotMode mode);

	void SetHangerState(HangerState hangerState);
	void SetAutoHang();

private:
	TaskMgr *m_scheduler;
	CANTalon *m_crankMotor;

	HangerState m_hangerState;

	double m_crankCurrent;
};

} /* namespace frc973 */
