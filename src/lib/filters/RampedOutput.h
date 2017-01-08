/*
 * RampedOutput.h
 *
 * RampedOutput is a filter that garauntees the maximum rate of change in any
 * signal is at most a certain threshold.  This can be used to ramp drive
 * output on a root with high CG, to ramp motor output on a flywheel that
 * might otherwise spike the battery, or as a filter to a sensor where
 * we know the rate of change of value should always be below a certain
 * threshold.
 *
 * The units for rampRate is whatever is being measured per sec.  This class
 * has an internal clock so regardless of how often it is being called, the
 * ramp rate will stay the same.
 *
 *  Created on: Oct 29, 2015
 *      Author: Andrew
 */

#ifndef LIB_RAMPEDOUTPUT_H_
#define LIB_RAMPEDOUTPUT_H_

#include <cstdint>
#include "lib/filters/FilterBase.h"

namespace frc973 {

class RampedOutput : public FilterBase {
public:
	/**
	 * Create a RampedOutput object. This object filters a signal by coercing
	 * the input value to within range of the old value +/- the ramp rate.
	 *
	 * @param rampRate maximum difference between two outputs (in units/sec)
	 * @param initialOutput (optional) the first value we ramp from
	 */
	RampedOutput(double rampRate, double initialOutput = 0.0);
	virtual ~RampedOutput();

	/**
	 * Get the filtered output value given the actual input.  Takes into
	 * account the given input, the previous output, the ramp rate, and the
	 * time since last call.
	 *
	 * @param input The target value that we should try to output
	 *
	 * @return whatever value was closer to the input
	 */
	double Update(double input) override;

	/**
	 * Get the previous value returned by this ramp.  Doesn't accoutn for
	 * the time since that value was calculated.
	 *
	 * @return whatever value was last returned by GetValue
	 */
	double GetLast() override {
		return m_prevOutput;
	}

	/**
	 * Check whether (with the most recent call of GetValue) we've reached our
	 * target output value
	 */
	bool IsRampFinished(void);

	/**
	 * Change the ramp rate
	 *
	 * @param newRampRate new ramp rate (in units/sec)
	 */
	void SetRampRate(double newRampRate);

	/**
	 * Get the ramp rate
	 *
	 * @return rampRate (in units/sec)
	 */
	double GetRampRate(void);

	/**
	 * Override any ramping and just set the damn previous output...
	 * The RampedOutput will ramp subsequent values as if that were the
	 * previous output.
	 */
	void OverridePrevOutput(double prevOutput);

private:
	double m_rampRate;
	double m_prevOutput;
	uint32_t m_prevTimeMs;
	bool m_rampFinished;
};

}

#endif /* LIB_RAMPEDOUTPUT_H_ */
