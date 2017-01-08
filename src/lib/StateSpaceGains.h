/*
 * StateSpaceGains.h
 *
 *  Created on: Jan 17, 2016
 *      Author: Andrew
 */

#ifndef LIB_STATESPACEGAINS_H_
#define LIB_STATESPACEGAINS_H_

namespace frc973 {

class StateSpaceGains {
public:
	double *m_A;
	int m_aSize;
	double *m_B;
	int m_bSize;
	double *m_C;
	int m_cSize;
	double *m_D;
	int m_dSize;
	double *m_L;
	int m_lSize;
	double *m_K;
	int m_kSize;
	double *m_uMax;
	int m_uMaxSize;
	double *m_uMin;
	int m_uMinSize;

	bool m_updatedP;

	StateSpaceGains(
			double *A, int aS, double *B, int bS, double *C, int cS,
			double *D, int dS, double *L, int lS, double *K, int kS,
			double *uMax, int uMaxS, double *uMin, int uMinS) {
		m_A = A;
		m_aSize = aS;
		m_B = B;
		m_bSize = bS;
		m_C = C;
		m_cSize = cS;
		m_D = D;
		m_dSize = dS;
		m_L = L;
		m_lSize = lS;
		m_K = K;
		m_kSize = kS;
		m_uMax = uMax;
		m_uMaxSize = uMaxS;
		m_uMin = uMin;
		m_uMinSize = uMinS;

		m_updatedP = false;
	}

	bool Updated() {
		bool r = m_updatedP;
		m_updatedP = false;
		return r;
	}
};

}

#endif /* LIB_STATESPACEGAINS_H_ */
