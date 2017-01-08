/*
 * CheesyDrive.cpp
 *
 *  Created on: Oct 30, 2015
 *      Author: Andrew
 */

#include <controllers/CheesyDriveController.h>
#include <math.h>

namespace frc973 {

CheesyDriveController::CheesyDriveController() :
		m_leftOutput(0.0),
		m_rightOutput(0.0),
		m_quickTurn(false),
		m_highGear(false),
		m_oldWheel(0.0),
		m_negInertiaAccumulator(0.0),
		m_quickStopAccumulator(0.0)
{
}

CheesyDriveController::~CheesyDriveController() {
}

void CheesyDriveController::CalcDriveOutput(DriveStateProvider *state,
		DriveControlSignalReceiver *out) {
	out->SetDriveOutput(m_leftOutput, m_rightOutput);
}

void CheesyDriveController::SetJoysticks(double throttle, double wheel) {
	bool isQuickTurn = m_quickTurn;
	/*
	 * Scale the turn input... range 0.0 to 1.0
	 * Numbers close to 0 mean linear response (feels unresponsive)
	 * Numbers close to 1 grow quickly (feels more responsive hard to control)
	 */
	double turnNonlinHigh = 0.2; // smaller is more responsive bigger is less responsive
	double turnNonlinLow = 0.8;
	double negInertiaHigh = 1.2; // bigger is more responsive
	double senseHigh = 1.2;
	double senseLow = 0.6;
	double senseCutoff = 0.1;
	double negInertiaLowMore = 2.5;
	double negInertiaLowLessExt = 5.0;
	double negInertiaLowLess = 3.0;
	double quickStopTimeConstant = 0.1;
	double quickStopLinearPowerThreshold = 0.2;
	double quickStopStickScalar = 0.8; //Bigger for faster stopping

	double wheelNonLinearity;

	double negInertia = wheel - m_oldWheel;
	m_oldWheel = wheel;

	/*
	 * Scale joystick input
	 */
	if (m_highGear) {
		wheelNonLinearity = turnNonlinHigh;
		// Apply a sin function that's scaled to make it feel better.
		wheel = sin(Constants::PI / 2.0 * wheelNonLinearity * wheel)
				/ sin(Constants::PI / 2.0 * wheelNonLinearity);
		wheel = sin(Constants::PI / 2.0 * wheelNonLinearity * wheel)
				/ sin(Constants::PI / 2.0 * wheelNonLinearity);
	}
	else {
		wheelNonLinearity = turnNonlinLow;
		// Apply a sin function that's scaled to make it feel better.
		wheel = sin(Constants::PI / 2.0 * wheelNonLinearity * wheel)
				/ sin(Constants::PI / 2.0 * wheelNonLinearity);
		wheel = sin(Constants::PI / 2.0 * wheelNonLinearity * wheel)
				/ sin(Constants::PI / 2.0 * wheelNonLinearity);
		wheel = sin(Constants::PI / 2.0 * wheelNonLinearity * wheel)
				/ sin(Constants::PI / 2.0 * wheelNonLinearity);
	}

	double leftPwm, rightPwm, overPower;
	double sensitivity = 1.7;

	double angularPower;
	double linearPower;

	// Negative inertia!
	double negInertiaScalar;
	if (m_highGear) {
		negInertiaScalar = negInertiaHigh;
		sensitivity = senseHigh;
	}
	else {
		if (wheel * negInertia > 0) {
			negInertiaScalar = negInertiaLowMore;
		}
		else {
			if (Util::abs(wheel) > 0.65) {
				negInertiaScalar = negInertiaLowLessExt;
			}
			else {
				negInertiaScalar = negInertiaLowLess;
			}
		}
		sensitivity = senseLow;

		if (Util::abs(throttle) > senseCutoff) {
			sensitivity = 1 - (1 - sensitivity) / Util::abs(throttle);
		}
	}

	double negInertiaPower = negInertia * negInertiaScalar;
	m_negInertiaAccumulator += negInertiaPower;

	wheel = wheel + m_negInertiaAccumulator;
	if(m_negInertiaAccumulator > 1)
		m_negInertiaAccumulator -= 1;
	else if (m_negInertiaAccumulator < -1)
		m_negInertiaAccumulator += 1;
	else
		m_negInertiaAccumulator = 0;

	linearPower = throttle;

	// Quickturn!
	if (isQuickTurn) {
		if (Util::abs(linearPower) < quickStopLinearPowerThreshold) {
			double alpha = quickStopTimeConstant;
			m_quickStopAccumulator = (1 - alpha) * m_quickStopAccumulator + alpha * Util::bound(wheel, -1.0, 1.0) * quickStopStickScalar;
		}
		overPower = 1.0;
		if (m_highGear) {
			sensitivity = 1.0;
		}
		else {
			sensitivity = 1.0;
		}
		angularPower = wheel;
	}
	else {
		overPower = 0.0;
		angularPower = Util::abs(throttle) * wheel * sensitivity - m_quickStopAccumulator;
		if (m_quickStopAccumulator > 1) {
			m_quickStopAccumulator -= 1;
		}
		else if (m_quickStopAccumulator < -1) {
			m_quickStopAccumulator += 1;
		}
		else {
			m_quickStopAccumulator = 0.0;
		}
	}

	rightPwm = leftPwm = linearPower;
	leftPwm += angularPower;
	rightPwm -= angularPower;

	if (leftPwm > 1.0) {
		rightPwm -= overPower * (leftPwm - 1.0);
		leftPwm = 1.0;
	}
	else if (rightPwm > 1.0) {
		leftPwm -= overPower * (rightPwm - 1.0);
		rightPwm = 1.0;
	}
	else if (leftPwm < -1.0) {
		rightPwm += overPower * (-1.0 - leftPwm);
		leftPwm = -1.0;
	}
	else if (rightPwm < -1.0) {
		leftPwm += overPower * (-1.0 - rightPwm);
		rightPwm = -1.0;
	}

	m_leftOutput = leftPwm;
	m_rightOutput = rightPwm;
}

void CheesyDriveController::SetQuickTurn(bool quickturn) {
	m_quickTurn = quickturn;
}

}
