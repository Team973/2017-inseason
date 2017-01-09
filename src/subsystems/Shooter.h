/*
 * Shooter.h
 *
 *  Created on: Oct 9, 2015
 *      Author: Andrew
 */

#ifndef SRC_SUBSYSTEMS_SHOOTER_H_
#define SRC_SUBSYSTEMS_SHOOTER_H_

#include "WPILib.h"
#include "lib/CoopTask.h"

using namespace frc;

namespace frc973 {

class TaskMgr;
class LogSpreadsheet;

/**
 * Open loop control on flywheel at the moment... will do fine tuning
 * once it's shown that everything else works
 */
class Shooter : public CoopTask
{
public:
	Shooter(TaskMgr *scheduler, LogSpreadsheet *logger);
	virtual ~Shooter();
	void TaskPeriodic(RobotMode mode);
	void SetFlywheelPow(double pow);
	void SetFlywheelStop();

	double GetFlywheelRate();

enum FlywheelState {
	running,
	notRunning
};

private:
	TaskMgr *m_scheduler;

	Talon *m_flywheelMotor;

	double m_flywheelPow;

	FlywheelState m_flywheelState;

	LogCell *m_shooterRate;
	LogCell *m_shooterPow;

	Encoder *m_flywheelEncoder;
};

}

#endif /* SRC_SUBSYSTEMS_SHOOTER_H_ */
