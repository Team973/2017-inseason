/*
 * Hanger.h
 *
 *  Created on: Apr 17, 2016
 *      Author: Andrew
 */

#ifndef SRC_SUBSYSTEMS_HANGER_H_
#define SRC_SUBSYSTEMS_HANGER_H_

class DoubleSolenoid;
class VictorSP;
class DigitalInput;

#include "lib/CoopTask.h"
#include "lib/TaskMgr.h"

namespace frc973 {

class Drive;
class Shooter;

class Hanger : public CoopTask {
public:
	enum HangerState {
		PreHanging,		//Before hanging
		AutoHanging,	//Hanging automatically (listening to halls)
		ManualHanging,	//Hanging manually
		PostHanging		//Hanging but not sending power
	};

	Hanger(TaskMgr *scheduler, Drive *drive, VictorSP *crankMotor, Shooter *shooter);
	virtual ~Hanger();

	/**
	 * Release hooks and start moving the crank to a specified position using
	 * closed loop control
	 *
	 * @param hang, true to start auto hang sequence, false to stop it
	 */
	void SetAutoHang(bool hang);

	/**
	 * Release hooks and move the crank with open loop full blast
	 *
	 * @param hang, true to start manual hang sequence, false to stop it
	 */
	void SetManualHang(bool hang);

	/**
	 * Release the hooks if they haven't already been released
	 * If they have been released, just do nothing (don't spam them)
	 */
	void TryReleaseHooks();

private:
	/**
	 * Take control of the conveyor motor away from the shooter.
	 */
	void TakeConveyorMotor();

	void TaskPeriodic(RobotMode mode);

	TaskMgr *m_scheduler;
	Drive *m_drive;
	Shooter *m_shooter;

	DoubleSolenoid *m_ptoRelease;
	VictorSP *m_crankMotor;

	DigitalInput *m_leftHookSensor;
	DigitalInput *m_rightHookSensor;

	bool m_hooksReleased;
	bool m_everSeenSwitch;

	HangerState m_state;
};

} /* namespace frc973 */

#endif /* SRC_SUBSYSTEMS_HANGER_H_ */
