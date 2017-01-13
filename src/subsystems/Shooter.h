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
#include "CANTalon.h"

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
	void SetFLywheelSpeed(double speed);

	double GetFlywheelRate();

enum FlywheelState {
	running,
	notRunning,
	speed
};

private:
	TaskMgr *m_scheduler;

	CANTalon *m_flywheelMotorPrimary;
	CANTalon *m_flywheelMotorReplica;

	double m_flywheelPow;
	double m_flywheelSpeed;

	FlywheelState m_flywheelState;

	LogCell *m_flywheelRate;
	LogCell *m_flywheelPowLog;

};

}

#endif /* SRC_SUBSYSTEMS_SHOOTER_H_ */
