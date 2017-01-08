/*
 * MovingAverageFilter.h
 *
 *  Created on: Oct 18, 2015
 *      Author: Andrew
 *
 * Simple data filter.  Given a value m between 0.0 and 1.0, return the
 * current datapoint times (1 - m) plus the previous datapoint times m.
 */

#ifndef LIB_MOVINGAVERAGEFILTER_H_
#define LIB_MOVINGAVERAGEFILTER_H_

#include "lib/filters/FilterBase.h"

namespace frc973 {

class MovingAverageFilter : public FilterBase {
public:
	/**
	 * Create a data filter by the moving average method.
	 *
	 * @param weight weight of the previous value when determining the
	 * filtered value
	 * @param initial value to consider as the previous value
	 */
	MovingAverageFilter(double weight, double initial = 0.0);
	virtual ~MovingAverageFilter();

	/**
	 * Calculate the filtered value given the original datapoint.
	 *
	 * @parm currentValue  the current data point that needs to be filtered
	 *
	 * @return result of filtering calculation.
	 */
	double Update(double input);

	/**
	 * Remember the latest value calculated by filtering
	 *
	 * @return result of previous filtering calcuation
	 */
	double GetLast(void);

private:
	double m_weight;
	double m_prevValue;
};

}

#endif /* LIB_MOVINGAVERAGEFILTER_H_ */
