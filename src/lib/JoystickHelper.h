/*
 * JoystickHelper.h
 *
 *  Created on: Oct 14, 2015
 *      Author: Andrew
 */

#ifndef LIB_JOYSTICKHELPER_H_
#define LIB_JOYSTICKHELPER_H_

#include <stdint.h>
#include "CoopTask.h"
#include "WPILib.h"

class DriverStation;

namespace frc973 {

/*
 * Button mapping for the DualAction joystick
 */
namespace DualAction {
	/*
	 * Standard buttons... shouldn't need any explanation
	 */
	const unsigned int BtnX = 1;
	const unsigned int BtnA = 2;
	const unsigned int BtnB = 3;
	const unsigned int BtnY = 4;
	const unsigned int LeftBumper = 5;
	const unsigned int RightBumper = 6;
	const unsigned int LeftTrigger = 7;
	const unsigned int RightTrigger = 8;
	const unsigned int Back = 9;
	const unsigned int Start = 10;

	/*
	 * When you push down on the left and right joystick, that registers
	 * as a button press
	 */
	const unsigned int LJoystickBtn = 11;
	const unsigned int RJoystickBtn = 12;

	const unsigned int DPadUpVirtBtn = 22;
	const unsigned int DPadDownVirtBtn = 23;
	const unsigned int DPadLeftVirtBtn = 24;
	const unsigned int DPadRightVirtBtn = 25;

	/*
	 * The following are 'virtual' buttons, one for each joystick axis.
	 *  * Virtual buttons default to zero.
	 *  * When you push the associated joystick axis above 0.5, it registers
	 *  	as pressed
	 *  * When you pull the associated joystick axis below -0.5, it registers
	 *  	as released
	 */
	const unsigned int LXAxisVirtButton = 26;
	const unsigned int LYAxisVirtButton = 27;
	const unsigned int RXAxisVirtButton = 28;
	const unsigned int RYAxisVirtButton = 29;
	const unsigned int DXAxisVirtButton = 30;
	const unsigned int DYAxisVirtButton = 31;

	/*
	 * Not buttons but the numbers for each axis... can be used with
	 * joystick.GetRawAxis
	 * DPad axis only return 0.0, -1.0, and 1.0
	 */
	const unsigned int LeftXAxis = 0;
	const unsigned int LeftYAxis = 1;
	const unsigned int RightXAxis = 2;
	const unsigned int RightYAxis = 3;
	const unsigned int DPadXAxis = 4;
	const unsigned int DPadYAxis = 5;
}

class ObservableJoystick;

/**
 * This abstract class defines the JoystickObserver object. The object is
 * a callback interface. It is not meant to be created as an object.
 * Instead, it should be inherited by a subclass who needs to be notified
 * on the joystick button events.
 */
class JoystickObserver {
public:
	JoystickObserver() {}
	virtual ~JoystickObserver() {}

    /**
     * This function is provided by the subclass to handle a joystick
     * button event notification.
     *
     * @param port Specifies the joystick port.
     * @param button Specifies the joystick button
     * @param newState If true, specifies the button has been pressed,
     *        if false, specifies the button has been released.
     */
	virtual void ObserveJoystickStateChange(
			uint32_t port,
			uint32_t button,
			bool newState
			) = 0;
};

/**
 * This class observes a given joystick and notifies the given callback
 * on any joystick event.  This is particularly useful for catching the
 * *edge* of a button press or release event.  Also lets you use a joystick
 * axis as a button in an easy way.
 */
class ObservableJoystick: public CoopTask,
						  public Joystick
{
public:
	static constexpr double DEADBAND_INPUT_THRESHOLD = 0.07;
	static constexpr double VIRTUAL_JOYSTICK_THRESHOLD = 0.5;

protected:
    uint32_t		m_port;

    /* For observer notification */
    JoystickObserver *m_observer;
    DriverStation  *m_ds;
    uint32_t        m_prevBtn;
    TaskMgr		   *m_scheduler;

    /* For remembering states of sticky buttons */
    bool m_lastLXVal;
    bool m_lastLYVal;
    bool m_lastRXVal;
    bool m_lastRYVal;
    bool m_lastDXVal;
    bool m_lastDYVal;

public:
    /**
     * Create an instance of the ObservableJoystick object.  Requires the
     * information to instantiate the underlying WPI-Joystick, as well as
     * references to the scheduler that will run it and the observer that
     * will observe its state.
     *
     * @param port Specifies the joystick port.
     * @param notify Points to the JoystickObserver object for button event
     *        notification callback.
     * @param scheduler Points to the task manager this task will run on
     */
    ObservableJoystick(uint16_t port, JoystickObserver *observer,
    		TaskMgr *scheduler, DriverStation *ds = nullptr);

    ~ObservableJoystick();

    /**
     * Get the value of the given axis with deadband.
     *
     * @param axis Specifies the axis to get the value of.
     * @param fSquared Specifies whether the joystick input should be squared.
     * @param threshold Specifies the deadband threshold.
     * @param hand Specifies the handedness of the joystick (default to right
     *        hand).
     */
    float GetRawAxisWithDeadband(int axis, bool fSquared = false,
    		float threshold = DEADBAND_INPUT_THRESHOLD);

    /*
     * Check whether the up button on the d pad is pressed
     */
    bool GetDPadUpVirtButton();

    /*
     * Check whether the down btton on the d pad is pressed
     */
    bool GetDPadDownVirtButton();

    /*
     * Check whetehr the left button on the d pad is pressed
     */
    bool GetDPadLeftVirtButton();

    /*
     * Check whether the right button on the d pad is pressed
     */
    bool GetDPadRightVirtButton();

    /**
     * Pretend the Left X Axis is a button.  By default it is not pressed.
     * If the user pushes it mostly forward (say, more than half way), say
     * that button is pressed.  If the user pulls it mostly backwards (say,
     * more than half way), say that button is released.  If it's anywhere
     * in between, rememember what it last was.
     *
     * @return whether the left X virtual button is pressed
     */
    bool GetLXVirtButton();

    /*
     * Left Y Virtual button
     */
    bool GetLYVirtButton();

    /*
     * Right X Virtual button
     */
    bool GetRXVirtButton();

    /**
     * Right Y Virtual button
     */
    bool GetRYVirtButton();

    /**
     * DPad X virtual button
     */
    bool GetDXVirtButton();

    /**
     * DPad Y Virtual button
     */
    bool GetDYVirtButton();

    /**
     * Get a bitstring containing the state of *all* buttons on the joystick.
     * Including any 'virtual' buttons like the 'joystick buttons'
     */
    uint32_t GetAllButtons();

    /**
     * This function is called by the TaskMgr to check and process Joystick
     * button events.
     *
     * @param mode Specifies the CoopTask callback types.
     */
    void TaskPrePeriodic(RobotMode mode) override;
};

}

#endif /* LIB_JOYSTICKHELPER_H_ */
