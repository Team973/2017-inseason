#include "controllers/StateSpaceFlywheelController.h"
#include "lib/util/Matrix.h"
#include "lib/filters/Debouncer.h"
#include "lib/util/Util.h"

#include "WPILib.h"

namespace frc973 {

StateSpaceFlywheelController::StateSpaceFlywheelController(StateSpaceGains *gains) :
	StateSpaceController(1, 1, 2, gains, 1.0 / 200.0) {
	enabled = true;
	velGoal = 0.0;
	y = new Matrix(1,1);
	r = new Matrix(2,1);
	prevPos = 0.0;
	goal = 0.0;
	curVel = 0.0;
	outputVoltage = 0.0;
	targetError = 9;
	filter = new Debouncer(1.0 / 200.0 * 4.0);
}

void StateSpaceFlywheelController::CapU() {
	double dU = U->Get(0);
	double uMax = UMax->Get(0);
	double uMin = UMin->Get(0);
	//double u = XHat->Get(0);

	double upWindow = uMax - outputVoltage;
	double dnWindow = uMin - outputVoltage;

	printf("dU %lf (uMax %lf uMin %lf) (upWindow %lf dnWindow %lf)\n",
			dU, uMax, uMin, upWindow, dnWindow);
	dU = Util::bound(dU, dnWindow, upWindow);

	outputVoltage += dU;
	U->Set(0, dU);
}

double StateSpaceFlywheelController::Update(double curSensorVel) {
	if (!enabled) {
		return 0.0;
	}

	curVel = curSensorVel;

	y->Set(0, 0, curSensorVel);

	r->Set(0, 0, (velGoal * (1 - A->Get(1, 1))) / A->Get(1, 0));
	r->Set(1, 0, velGoal);

	UpdateCont(r, y);
	CapU();

	double voltage = DriverStation::GetInstance().GetBatteryVoltage();
	if (voltage < 4.5) {
		voltage = 12.0;
	}

	printf("velGoal %lf, curVel %lf, outputV %lf\n",
			velGoal, curVel, outputVoltage);
	if (velGoal < 1.0) {
		goal = curSensorVel;
		return 0.0;
	}
	else if (outputVoltage < 0.0) {
		return 0.0;
	}
	else {
		return (outputVoltage / voltage) * 1.0;
	}
}

double StateSpaceFlywheelController::GetVelocity() {
	return curVel;
}

void StateSpaceFlywheelController::SetVelocityGoal(double v) {
	velGoal = v;
}

void StateSpaceFlywheelController::Enable() {
	enabled = true;
}

void StateSpaceFlywheelController::Disable() {
	curVel = 0.0;
	enabled = false;
}

}
