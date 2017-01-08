#include "JoystickHelper.h"
#include "TaskMgr.h"

namespace frc973 {

ObservableJoystick::ObservableJoystick(
	uint16_t port,
	JoystickObserver *observer,
	TaskMgr *scheduler,
	DriverStation *ds
	): Joystick(port)
	 , m_port(port)
	 , m_observer(observer)
	 , m_ds(ds)
	 , m_prevBtn(0)
	 , m_scheduler(scheduler)
	 , m_lastLXVal(false)
	 , m_lastLYVal(false)
	 , m_lastRXVal(false)
	 , m_lastRYVal(false)
	 , m_lastDXVal(false)
	 , m_lastDYVal(false)
{
	if (m_ds == nullptr) {
		m_ds = &DriverStation::GetInstance();
	}

	m_prevBtn = m_ds->GetStickButtons(port);

	if (scheduler != nullptr) {
		scheduler->RegisterTask("JoystickHelper", this, TASK_PRE_PERIODIC);
	}
}

ObservableJoystick::~ObservableJoystick() {
	if (m_scheduler != nullptr) {
		m_scheduler->UnregisterTask(this);
	}
}

float ObservableJoystick::GetRawAxisWithDeadband(int axis, bool fSquared,
		float threshold) {
	float value = Util::deadband(GetRawAxis(axis), threshold);

	if (fSquared) {
		value = Util::signSquare(value);
	}

	return value;
}

bool ObservableJoystick::GetDPadUpVirtButton() {
	int pov = GetPOV();
	return pov == 0 || pov == 315 || pov == 45;
}

bool ObservableJoystick::GetDPadDownVirtButton() {
	int pov = GetPOV();
	return pov == 180 || pov == 225 || pov == 135;
}

bool ObservableJoystick::GetDPadLeftVirtButton() {
	int pov = GetPOV();
	return pov == 270 || pov == 315 || pov == 225;
}

bool ObservableJoystick::GetDPadRightVirtButton() {
	int pov = GetPOV();
	return pov == 90 || pov == 135 || pov == 45;
}

bool ObservableJoystick::GetLXVirtButton() {
	double pos = this->GetRawAxis(DualAction::LeftXAxis);

	if (pos > VIRTUAL_JOYSTICK_THRESHOLD) {
		m_lastLXVal = true;
	} else if (pos < -VIRTUAL_JOYSTICK_THRESHOLD) {
		m_lastLXVal = false;
	}

	return m_lastLXVal;
}

bool ObservableJoystick::GetLYVirtButton() {
	double pos = -this->GetRawAxis(DualAction::LeftYAxis);

	if (pos > VIRTUAL_JOYSTICK_THRESHOLD) {
		m_lastLYVal = true;
	} else if (pos < -VIRTUAL_JOYSTICK_THRESHOLD) {
		m_lastLYVal = false;
	}

	return m_lastLYVal;
}

bool ObservableJoystick::GetRXVirtButton() {
	double pos = this->GetRawAxis(DualAction::RightXAxis);

	if (pos > VIRTUAL_JOYSTICK_THRESHOLD) {
		m_lastRXVal = true;
	} else if (pos < -VIRTUAL_JOYSTICK_THRESHOLD) {
		m_lastRXVal = false;
	}

	return m_lastRXVal;
}

bool ObservableJoystick::GetRYVirtButton() {
	double pos = -this->GetRawAxis(DualAction::RightYAxis);

	if (pos > VIRTUAL_JOYSTICK_THRESHOLD) {
		m_lastRYVal = true;
	} else if (pos < -VIRTUAL_JOYSTICK_THRESHOLD) {
		m_lastRYVal = false;
	}

	return m_lastRYVal;
}

bool ObservableJoystick::GetDXVirtButton() {
	if (this->GetDPadRightVirtButton()) {
		m_lastDXVal = true;
	} else if (this->GetDPadLeftVirtButton()) {
		m_lastDXVal = false;
	}

	return m_lastDXVal;
}

bool ObservableJoystick::GetDYVirtButton() {
	if (this->GetDPadUpVirtButton()) {
		m_lastDYVal = true;
	} else if (this->GetDPadDownVirtButton()) {
		m_lastDYVal = false;
	}

	return m_lastDYVal;
}

uint32_t ObservableJoystick::GetAllButtons() {
	uint32_t btns = m_ds->GetStickButtons(m_port);

	btns |= GetLXVirtButton() << (DualAction::LXAxisVirtButton - 1);
	btns |= GetLYVirtButton() << (DualAction::LYAxisVirtButton - 1);
	btns |= GetRXVirtButton() << (DualAction::RXAxisVirtButton - 1);
	btns |= GetRYVirtButton() << (DualAction::RYAxisVirtButton - 1);
	btns |= GetDXVirtButton() << (DualAction::DXAxisVirtButton - 1);
	btns |= GetDYVirtButton() << (DualAction::DYAxisVirtButton - 1);
	btns |= GetDPadUpVirtButton() << (DualAction::DPadUpVirtBtn - 1);
	btns |= GetDPadDownVirtButton() << (DualAction::DPadDownVirtBtn - 1);
	btns |= GetDPadLeftVirtButton() << (DualAction::DPadLeftVirtBtn - 1);
	btns |= GetDPadRightVirtButton() << (DualAction::DPadRightVirtBtn - 1);

	return btns;
}

/**
 * Careful this code is dense and contains crazy bit-shifty logic.
 *    X&~(X^-X) extracts the least significant set bit from X in mask form
 *    __builtin_ffs(Y) gets the position of the least significant set bit
 */
void ObservableJoystick::TaskPrePeriodic(RobotMode mode) {
	uint32_t currBtn = GetAllButtons();

	if (m_observer != nullptr) {
		uint32_t changedBtn = m_prevBtn^currBtn;
		uint32_t btnMask, btn;

		while (changedBtn != 0) {
			/* btnMask contains the least significant set bit in changedBtn */
			btnMask = changedBtn & ~(changedBtn^-changedBtn);
			/* btn contains the index of the lssb in btnMask... aka the button number */
			btn = __builtin_ffs(btnMask);
			if ((currBtn & btnMask) != 0) {
				/* Button is pressed */
				m_observer->ObserveJoystickStateChange(m_port, btn, true);
			}
			else {
				/* Button is released */
				m_observer->ObserveJoystickStateChange(m_port, btn, false);
			}
			/* clear |changedBtn| from the mask so we can get the next lsb */
			changedBtn &= ~btnMask;
		}
	}
	m_prevBtn = currBtn;
}

}
