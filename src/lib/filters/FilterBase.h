/*
 * FilterBase.h
 *
 *  Created on: Feb 23, 2016
 *      Author: Andrew
 */

#ifndef LIB_FILTERS_FILTERBASE_H_
#define LIB_FILTERS_FILTERBASE_H_

namespace frc973 {

class FilterBase {
public:
	/**
	 * Interface for a filter.  If all filters implement this interface, they
	 * can be passed around generically (like in the cascading filter filter).
	 */
	FilterBase();
	virtual ~FilterBase();

	/**
	 * Filter the given value and return the filterd value
	 *
	 * @param input value to filter
	 * @return filtered value
	 */
	virtual double Update(double input) = 0;
	virtual double GetLast() = 0;
};

}

#endif /* LIB_FILTERS_FILTERBASE_H_ */
