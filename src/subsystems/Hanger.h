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
#include "CANTalon.h"

using namespace frc;

namespace frc973 {

class Hanger : public CoopTask {
public:
	enum HangerState {
		autoHang,
		preHang,
		postHang
	};

	Hanger(TaskMgr *scheduler);
	virtual ~Hanger();
	void TaskPeriodic(RobotMode mode);

	void SetPreHang();
	void ReleaseAutoHang();
	void SetPostHang();

private:
	TaskMgr *m_scheduler;
	CANTalon *m_crankMotor;
	DoubleSolenoid *m_ptoRelease;

	bool m_hookReleased;

	HangerState m_hangerState;
};

} /* namespace frc973 */

#endif /* SRC_SUBSYSTEMS_HANGER_H_ */
