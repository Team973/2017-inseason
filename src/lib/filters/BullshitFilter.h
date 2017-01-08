/*
 * BullshitFilter.h
 *
 * Bullshit filter looks at a signal and calls bullshit when a point is
 * outside of a valid range.  The behavior when the filter sees a point
 * outside of that range is configurable.
 *
 *  Created on: Feb 29, 2016
 *      Author: andrew
 */

#ifndef LIB_FILTERS_BULLSHITFILTER_H_
#define LIB_FILTERS_BULLSHITFILTER_H_

#include "lib/filters/FilterBase.h"

namespace frc973 {

class BullshitFilter : public FilterBase {
public:
	/**
	 * What do we do for absurdly high values?
	 *  - noMax -> there is no cap... positive infinity is just gucci
	 *  - clipMax -> if the input is greater than the max, just go with the max
	 *  - dropMax -> if the input is greater than the max, use the previous acceptable value
	 */
	enum MaxBehavior {
		noMax,
		clipMax,
		dropMax
	};

	enum MinBehavior {
		noMin,
		clipMin,
		dropMin
	};

	/**
	 * Construct a bullshit filter.  |minBehavior| specifies the behavior
	 * for when the value is less than |min|.  |maxBehavior| specifies the
	 * behavior for when the value is more than |max|.
	 */
	BullshitFilter(MinBehavior minBehavior, double min,
			MaxBehavior maxBehavior, double max);
	virtual ~BullshitFilter();

	double Update(double in) override;
	double GetLast() override;
private:
	MinBehavior m_minBehavior;
	double m_min;

	MaxBehavior m_maxBehavior;
	double m_max;

	double m_last;
};

} /* namespace frc973 */

#endif /* LIB_FILTERS_BULLSHITFILTER_H_ */
