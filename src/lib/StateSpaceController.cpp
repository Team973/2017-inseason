#include "lib/StateSpaceController.h"
#include "lib/util/Matrix.h"
#include "StateSpaceGains.h"
#include "util/Util.h"

namespace frc973 {

StateSpaceController::StateSpaceController(int nIn, int nOut, int nStates,
		StateSpaceGains *gains, double period) {
	m_numInputs = nIn;
	m_numOutputs = nOut;
	m_numStates = nStates;
	m_gains = gains;
	m_period = period;

	A = new Matrix(m_numStates, m_numStates);
	B = new Matrix(m_numStates, m_numOutputs);
	C = new Matrix(m_numOutputs, m_numStates);
	D = new Matrix(m_numOutputs, m_numOutputs);
	L = new Matrix(m_numStates, m_numOutputs);
	K = new Matrix(m_numOutputs, m_numStates);
	X = new Matrix(m_numStates, 1);
	XHat = new Matrix(m_numStates, 1);
	U = new Matrix(m_numOutputs, 1);
	Uuncapped = new Matrix(m_numOutputs, 1);
	UMin = new Matrix(m_numOutputs, 1);
	UMax = new Matrix(m_numOutputs, 1);

	UpdateGains();
}

void StateSpaceController::UpdateCont(Matrix *R, Matrix *Y) {
	if (m_gains->Updated()) {
		UpdateGains();
	}

	Matrix *r1 = Matrix::Subtract(R, XHat);

	delete U;
	U = Matrix::Multiply(K, r1);

	Uuncapped->Flash(U->GetData(), U->GetHeight());
	CapU();

	Matrix *b_u = Matrix::Multiply(B, U);
	Matrix *l_y = Matrix::Multiply(L, Y);
	Matrix *l_c = Matrix::Multiply(L, C);
	Matrix *a_lc = Matrix::Subtract(A, l_c);
	Matrix *alc_xhat = Matrix::Multiply(a_lc, XHat);
	Matrix *xhatp1 = Matrix::Add(alc_xhat, l_y);

	delete XHat;
	XHat = Matrix::Add(xhatp1, b_u);

	delete r1;
	delete b_u;
	delete l_y;
	delete l_c;
	delete a_lc;
	delete alc_xhat;
	delete xhatp1;
}

void StateSpaceController::UpdateGains() {
	A->Flash(m_gains->m_A, m_gains->m_aSize);
	B->Flash(m_gains->m_B, m_gains->m_bSize);
	C->Flash(m_gains->m_C, m_gains->m_cSize);
	D->Flash(m_gains->m_D, m_gains->m_dSize);
	K->Flash(m_gains->m_K, m_gains->m_kSize);
	L->Flash(m_gains->m_L, m_gains->m_lSize);
	UMin->Flash(m_gains->m_uMin, m_gains->m_uMinSize);
	UMax->Flash(m_gains->m_uMax, m_gains->m_uMaxSize);
}

void StateSpaceController::CapU() {
	for (int i = 0; i < m_numOutputs; i++) {
		double u_i = U->Get(i);
		double u_max = UMax->Get(i);
		double u_min = UMin->Get(i);

		U->Set(i,
				Util::bound(u_i, u_min, u_max));
	}
}

}
