#pragma once

/**
 * Interpolated lookup table
 */

namespace frc973 {

#include <vector>

class InterpLookupTable {
public:
    InterpLookupTable();
    virtual ~InterpLookupTable();

    /**
     * Add the (x, y) point to the table and return |this|
     * for chaining
     */
    InterpLookupTable &AddPoint(double x, double y);

    /**
     * Lookup the y associated with the given x.  If a point with
     * the given x is not in the table, interpolate between the two
     * nearest points
     */
    double LookupPoint(double x);

private:
    struct Point {
        double x, y;
    };

    std::vector<Point> points;
};

}
