#ifndef FRCLIB_UTIL_H_
#define FRCLIB_UTIL_H_

#include <stdint.h>
#include <math.h>

class RobotStateInterface;

uint64_t GetFPGATime();

namespace frc973 {

//#include "WPILib.h"

/**
 * Used internally to represent the state of the robot
 */
enum RobotMode {
    MODE_DISABLED,
    MODE_AUTO,
    MODE_TELEOP,
	MODE_TEST
};
extern const char *robotModes[];

/**
 * GetRobotModeString queries the default RobotStateInterface,
 * and returns a c-string representing the current robot mode.
 *
 * @return string representing current robot mode
 */
const char *GetRobotModeString();

/**
 * GetRobotMode queries the default RobotStateInterface, and
 * returns the mode the robot is running in.
 *
 * @return returns current robot mode in RobotMode format
 */
RobotMode GetRobotMode();

/**
 * GetRobotMode queries the provided RobotStateInterface and returns
 * the mode the robot is running in.
 *
 * @param stateProvider RobotStateInterface to query to get running mode
 *
 * @return returns current robot mode in RobotMode format
 */
RobotMode GetRobotMode(RobotStateInterface &stateProvider);


/**
 * Constants
 */
namespace Constants {
	constexpr double PI = 3.141592653589793;
	constexpr double FEET_PER_METER = 3.280839895;
	constexpr double METERS_PER_FOOT = 1.0 / FEET_PER_METER;
	constexpr double GRAVITY_CONSTANT = 9.80665; // meter/sq(sec)
	constexpr double GRAVITY_CONSTANT_INCHES =
			GRAVITY_CONSTANT * FEET_PER_METER * 12.0;

	constexpr double USEC_PER_MSEC = 1000.0;
	constexpr double MSEC_PER_USEC = 1.0 / USEC_PER_MSEC;

	constexpr double MSEC_PER_SEC = 1000.0;
	constexpr double SEC_PER_MSEC = 1.0 / MSEC_PER_SEC;

	constexpr double USEC_PER_SEC = USEC_PER_MSEC * MSEC_PER_SEC;
	constexpr double SEC_PER_USEC = 1.0 / USEC_PER_SEC;

	constexpr double MIN_PER_SEC = 1.0 / 60.0;
	constexpr double SEC_PER_MIN = 60.0;

	constexpr double RAD_PER_DEG = 2 * PI / 360.0;
	constexpr double DEG_PER_RAD = 1.0 / RAD_PER_DEG;
}

/**
 * Macros
 */
#define ARRAYSIZE(a)            (sizeof(a)/sizeof((a)[0]))

inline double magnitude(double x,double y) {
	return sqrt(pow(x, 2.0) + pow(y, 2.0));
}

/* Get the current timestamp in microseconds */
inline uint64_t GetUsecTime() {
	return GetFPGATime();
}

/* Get the current time in milliseconds */
inline uint32_t GetMsecTime() {
	return GetUsecTime() * Constants::MSEC_PER_USEC;
}

/* Get the current time in seconds */
inline double GetSecTime() {
	return GetUsecTime() * Constants::SEC_PER_USEC;
}

namespace Util {
	/* Return |val| coerced to be above |low| and below |high| inclusive */
	inline double bound(double val, double low, double high) {
		if (val < low) {
			return low;
		}
		else if (val > high) {
			return high;
		}
		else {
			return val;
		}
	}

	/* Return the lesser of the two given numbers */
	inline double min(double a, double b) {
		if (a < b) {
			return a;
		}
		else {
			return b;
		}
	}

	/* Return the greater of the two given numbers */
	inline double max(double a, double b) {
		if (a > b) {
			return a;
		}
		else {
			return b;
		}
	}

	/* Return the absolute of the given number */
	inline double abs(double x) {
		if (x > 0.0) {
			return x;
		}
		else {
			return -x;
		}
	}

	/**
	 * Return 0 if |n| is within +/- |threshold|, otherwise return |n|
	 * Useful for joysticks that aren't quite centered at zero
	 */
	inline double deadband(double n, double threshold) {
		if (abs(n) > threshold) {
			return n;
		}
		else {
			return 0.0;
		}
	}

	/**
	 * Return |n| if abs(n) > threshold... otherwise return threshold
	 * with the sign of |n|.
	 *
	 * If |n| is 0.0, return |threshold|.
	 */
	inline double antideadband(double n, double threshold) {
		if (abs(n) < threshold) {
			if (n < 0.0) {
				return -threshold;
			}
			else {
				return threshold;
			}
		}
		else {
			return n;
		}
	}

	/**
	 * Increase the given number, |x|, by |threshold| but respeting the sign
	 * of |x|.  If |x| is positive, increase it, it |x| is negative, decrease
	 * it.
	 */
	inline double signedIncrease(double x, double increase) {
		if (x >= 0.0) {
			return x + increase;
		}
		else {
			return x - increase;
		}
	}

	/**
	 * Square the given number, but keep the sign the same
	 */
	inline double signSquare(double n) {
		if (n < 0.0) {
			return -1.0 * n * n;
		}
		else {
			return n * n;
		}
	}

	/* returns true if a and b are close (within epsilon) to each other */
	inline bool close(double a, double b, double epsilon = 0.00001) {
		return abs(a - b) < epsilon;
	}

	/*
	 * The NORMALIZE macro transforms a value (n) in the range between (sl) and
	 * (sh) to the range between (tl) and (th).
	*/
	inline double normalize(double n, double sl, double sh,
			double tl, double th) {
		return (n - sl) * (th - tl) / (sh - sl) + tl;
	}
}

}

#endif	/* FRCLIB_UTIL_H_ */
