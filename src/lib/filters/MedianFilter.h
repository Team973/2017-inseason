/*
 * MedianFilter.h
 *
 *  Created on: Jan 17, 2016
 *      Author: Andrew
 */

#pragma once

#include "FilterBase.h"

namespace frc973 {

class MedianFilter : public FilterBase {
public:
	MedianFilter(int buffSize = 5);
	virtual ~MedianFilter();

	double Update(double in) override;
	double GetLast() override;
private:
	int m_buffSize;
	double *m_samples;
	int m_idx;
	double m_last;
};

}
