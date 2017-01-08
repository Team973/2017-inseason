/*
 * StateSpaceController.h
 * Blatant copy of 254's state space controller
 * https://github.com/Team254/FRC-2014/blob/master/src/com/team254/lib/StateSpaceController.java
 * #noshame
 */

#ifndef LIB_STATESPACECONTROLLER_H_
#define LIB_STATESPACECONTROLLER_H_

namespace frc973 {

class Matrix;
class StateSpaceGains;

class StateSpaceController {
public:
	int m_numInputs;
	int m_numOutputs;
	int m_numStates;

	bool m_initP;

	StateSpaceGains *m_gains;

	double m_period;

	Matrix *A;
	Matrix *B;
	Matrix *C;
	Matrix *D;
	Matrix *L;
	Matrix *K;

	Matrix *X;
	Matrix *XHat;
	Matrix *U;
	Matrix *Uuncapped;
	Matrix *UMin;
	Matrix *UMax;

	StateSpaceController(int nIn, int nOut, int nStates,
			StateSpaceGains *gains, double period);

	void UpdateCont(Matrix *R, Matrix *Y);

private:
	void UpdateGains();

	void CapU();
};

}

#endif /* LIB_STATESPACECONTROLLER_H_ */
