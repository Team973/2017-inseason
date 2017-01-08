/*
 * ControllerBase.h
 *
 * Defines a few related classes for creating control systems in an object
 * oriented manor.  Yes, we could merge this with the controller classes
 * in DriveBase using templates, but the frc toolchain doesn't have fantastic
 * support for templates (and we're trying to keep compile time down :p).
 *
 * Am I really solving a problem here?  Let's just try the old-style of
 * organizing controls and see if it gets messy.
 *
 *  Created on: Jan 28, 2016
 *      Author: andrew
 */

#ifndef LIB_CONTROLLERBASE_H_
#define LIB_CONTROLLERBASE_H_

#include "CoopTask.h"
#include "TaskMgr.h"

namespace frc973 {

class SimpleControlSystem;

/**
 * Interface for something that can tell the current state of the system.
 * Used by the controller to get its input signal
 */
class SimpleControlStateProvider {
	SimpleControlStateProvider() {}
	virtual ~SimpleControlStateProvider() {}

	virtual double GetRate(SimpleControlSystem *system = nullptr) = 0;
	virtual double GetPosition(SimpleControlSystem *system = nullptr) = 0;
};

/**
 * Interface for something that can send signal to the system (power motors).
 * The controller sends its output signal here
 */
class SimpleControlSignalReceiver {
	SimpleControlSignalReceiver() {}
	virtual ~SimpleControlSignalReceiver() {}

	/**
	 * Receive calculated motor powers from a controller.
	 * Should only be called from a child of SimpleController.
	 */
	virtual void SetControllerOutput(double output,
			SimpleControlSystem *system = nullptr) = 0;
};

/**
 * Interface for a simple controller.  One that receives system state from
 * a SimpleControlStateProvider and sends output to a
 * SimpleControlSignalReceiver.  This could be generalized with templates,
 * but this is easier to debug.
 */
class SimpleController {
public:
	SimpleController(SimpleControlSystem *system);
	virtual ~SimpleController();

	virtual void Enable();
	virtual void Disable();

	/**
	 * Use the input signals from |angle| and |dist| and calculate some output,
	 * then send that output to |out|.
	 */
	virtual void CalcControllerOutput(SimpleControlStateProvider *state,
			SimpleControlSignalReceiver *out) = 0;
	/**
	 * Check whether the controller thinks we are on target.
	 */
	virtual bool OnTarget() = 0;
};

class SimpleControlSystem : public CoopTask {
public:
	SimpleControlSystem(TaskMgr *scheduler,
			SimpleControlStateProvider *state,
			SimpleControlSignalReceiver *out,
			SimpleController *controller = nullptr) :
				m_scheduler(scheduler),
				m_state(state),
				m_out(out),
				m_activeController(controller) {
		m_scheduler->RegisterTask("Simple Control System", this, TASK_POST_PERIODIC);
	}
	virtual ~SimpleControlSystem() {
		m_scheduler->UnregisterTask(this);
	}

	void SetActiveController(SimpleController *controller) {
		if (m_activeController) {
			m_activeController->Disable();
		}
		m_activeController = controller;
		m_activeController->Enable();
	}

	bool OnTarget() {
		if (!m_activeController) {
			return false;
		}
		return m_activeController->OnTarget();
	}

	void TaskPostPeriodic(RobotMode mode) override {
		if (m_activeController != nullptr) {
			m_activeController->CalcControllerOutput(m_state, m_out);
		}
	}

private:
	TaskMgr *m_scheduler;
	SimpleControlStateProvider *m_state;
	SimpleControlSignalReceiver *m_out;
	SimpleController *m_activeController;
};

}

#endif /* LIB_CONTROLLERBASE_H_ */
