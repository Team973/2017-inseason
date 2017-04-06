/*
 * DriveBase.h
 *
 *  Created on: Oct 29, 2015
 *      Author: Andrew
 */

#pragma once

#include "lib/TaskMgr.h"
#include "lib/CoopTask.h"
#include "lib/util/Util.h"
#include "WPILib.h"
#include "CANTalon.h"

using namespace frc;

namespace frc973 {

/*
 * Interface for a class can determine the current statae of the drive
 * plant
*/
class DriveStateProvider {
public:
	DriveStateProvider() {}
	virtual ~DriveStateProvider() {}
	virtual double GetAngle() const = 0;
	virtual double GetAngularRate() const = 0;
	virtual double GetLeftDist() const = 0;
	virtual double GetRightDist() const = 0;
	virtual double GetLeftRate() const = 0;
	virtual double GetRightRate() const = 0;
	virtual double GetDist() const = 0;
	virtual double GetRate() const = 0;
};

/*
 * Interface for a class that can take drive output
*/
class DriveControlSignalReceiver {
public:
	DriveControlSignalReceiver() {}
	virtual ~DriveControlSignalReceiver() {}
	/**
	 * Receive calculated motor powers from a controller.
	 * Should only be called from a child of DriveController.
	 */
	virtual void SetDriveOutput(double left, double right) = 0;
	virtual void SetDriveControlMode(CANSpeedController::ControlMode mode) = 0;
};

/*
 * Interface for anything that can use an angle provider, a dist provider, and
 * can use those to send some drive output to the DriveOutput
 *
 * In general there will be two drive controllers: One that calculates output
 * based solely on joystick values and one that calcualtes output based solely
 * on pid.
*/
class DriveController {
public:
	DriveController() {}
	virtual ~DriveController() {}
	/**
	 * Use the input signals from |angle| and |dist| and calculate some output,
	 * then send that output to |out|.
	 */
	virtual void CalcDriveOutput(DriveStateProvider *state, DriveControlSignalReceiver *out) = 0;
	/**
	 * Check whether the controller thinks we are on target.
	 */
	virtual bool OnTarget() = 0;

	virtual void Start() {}

	virtual void Stop() {}
};

/**
 * (abstract) Base class for a robot drive.  DriveBase keeps track of one
 * DriveController and uses it to calculate drive output, then drives the
 * motors with those calculated values.
 *
 * CoopTask handles calling TaskPostPeriodic once a cycle
 */
class DriveBase :
		public CoopTask
{
public:
	/**
	 * Creates a new DriveBase Object.  The DriveBase object stores a drive
	 * controller (an object capable of calculating motor outputs) and uses it
	 * to calculate drive outputs, then drive those drive outputs.
	 */
	DriveBase(TaskMgr *scheduler, DriveStateProvider *state,
			DriveControlSignalReceiver *outpt,
			DriveController *controller = nullptr);
	virtual ~DriveBase();
    /**
     * When making calls to PID-like-commands, parameters can be relative to
     * the world (Absolute), relative to the current position (Now), or
     * relative to the current setpoint (SetPoint)
     */
    enum RelativeTo {
    	Absolute,
		Now,
		SetPoint
    };

	/*
	 * Get input from the currently active DriveController.  This method comes
	 * from CoopTask and is called automatically once a cycle by m_scheduler
	 *
	 * @param mode The current operating mode of the robot
	 */
	void TaskPostPeriodic(RobotMode mode) override;

	/*
	 * Change the DriveController currently active
	 *
	 * @param new controller to have active
	 */
	void SetDriveController(DriveController *controller);

	/*
	 * Checks with the current controller to see if we are on target.  If
	 * there is no controller currently selected, just return false
	 *
	 * @return whether the current controller things we are done
	 */
	bool OnTarget();
private:
	TaskMgr *m_scheduler;

	DriveStateProvider *m_stateProvider;
	DriveControlSignalReceiver *m_driveOutput;

	DriveController *m_controller;
};

}
