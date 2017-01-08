/*
 * CoopMTRobot.h
 *
 *  Created on: Sep 1, 2015
 *      Author: Andrew
 *
 * Functionality pretty similar to WPILIB's Iterative Robot
 * except that this one manages tasks and has a Start and Stop
 * method for each state.
 *
 * A more ideal solution would probably have been to inherit from
 * SimpleRobot and use the same virtual method names as Iterative Robot,
 * but IterativeRobot handles driver station and HAL communication and those
 * protocols are subject to change... this should be more maintainable
 * in the long run.
 */

#ifndef FRCLIB_COOPMTROBOT_H_
#define FRCLIB_COOPMTROBOT_H_

#include "stdint.h"
#include "WPILib.h"
#include "TaskMgr.h"
#include "util/Util.h"
#include <pthread.h>

#ifndef PROGRAM_NAME
#define PROGRAM_NAME "(unspecified)"
#endif

namespace frc973 {

static constexpr int MAXHOSTNAMELEN = 128;

class CoopMTRobot:
	public IterativeRobot,
	public TaskMgr,
	public RobotStateInterface
{
public:
	CoopMTRobot();
	virtual ~CoopMTRobot();

	/**
	 * Similar to RobotInit in the IterativeRobot class, override Initialize
	 * to do any post-constructor initialization.
	 */
	virtual void Initialize(void) {}

	/**
	 * The following {Disabled,Autonomous,Teleop,Test}{Start,Stop,Continuous}
	 * should be overridden to get behavior similar to overriding those functions
	 * in Iterative Robot.
	 *
	 * <Mode>Start is called when robot first changes into this mode
	 * 		(like <Mode>Init in IterativeRobot)
	 * <Mode>Stop is called when robot moves to another mode from Mode
	 * <Mode>Continuous is called repeatedly every 20ms
	 * 		(like <Mode>Periodic in IterativeRobot)
	 *
	 * <Mode>Start is garaunteed to be called before <Mode>Continuous
	 * <Mode>Stop for the previous mode is garaunteed to be called before
	 * 		<Mode>Start from the prevous mode
	 */
	virtual void DisabledStart(void) {}
	virtual void DisabledStop(void) {}
	virtual void DisabledContinuous(void) {}

	virtual void AutonomousStart(void) {}
	virtual void AutonomousStop(void) {}
	virtual void AutonomousContinuous(void) {}

	virtual void TeleopStart(void) {}
	virtual void TeleopStop(void) {}
	virtual void TeleopContinuous(void) {}

	virtual void TestStart(void) {}
	virtual void TestStop(void) {}
	virtual void TestContinuous(void) {}

	/**
	 * Called continuously during all robot stages
	 */
	virtual void AllStateContinuous(void) {}

protected:
	/**
	 * For internal use only.  Children of this object should not try to
	 * override these (if they do, they *WILL NOT GET RUN*).
	 */
	void RobotInit(void);

	void DisabledInit(void);
	void AutonomousInit(void);
	void TeleopInit(void);
	void TestInit(void);

	void DisabledPeriodic(void);
	void AutonomousPeriodic(void);
	void TeleopPeriodic(void);
	void TestPeriodic(void);

	void ModeStop(RobotMode toStop);
	void ModeStart(RobotMode toStart);

	/**
	 * Implement the RobotStateInterface interface so that we may
	 * cache robor mode.
	 */
	bool IsDisabled() const;
	bool IsEnabled() const;
	bool IsOperatorControl() const;
	bool IsAutonomous() const;
	bool IsTest() const;
private:
	RobotMode m_prevMode;
	mutable pthread_mutex_t m_robotModeMutex;
};

}

#endif /* FRCLIB_COOPMTROBOT_H_ */
