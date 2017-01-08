/*
 * CascadingFilter.h
 *
 * A Cascading filter contains a list of filters and applies them sequentially
 * to the signal.
 *
 *  Created on: Feb 29, 2016
 *      Author: andrew
 */

#ifndef LIB_FILTERS_CASCADINGFILTER_H_
#define LIB_FILTERS_CASCADINGFILTER_H_

#include "FilterBase.h"
#include <vector>

namespace frc973 {

class CascadingFilter : FilterBase {
public:
	CascadingFilter();
	virtual ~CascadingFilter();

	void PushFilter(FilterBase *newFilt);

	double Update(double in) override;

	double GetLast() override;
private:
	double m_last;
	std::vector<FilterBase*> m_children;
};

} /* namespace frc973 */

#endif /* LIB_FILTERS_CASCADINGFILTER_H_ */
