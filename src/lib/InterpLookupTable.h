#pragma once

/**
 * Interpolated lookup table
 */

#include <vector>

namespace frc973 {

class InterpLookupTable {
public:
    InterpLookupTable();
    virtual ~InterpLookupTable();

    /**
     * Add the (x, y) point to the table and return |this|
     * for chaining
     * x: pixy height offset; y: horizontal distance
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
        bool operator<(const Point &rhs) const {
            return x < rhs.x;
        }
    };

    double InterpolatePoints(const Point &a, const Point &b, double x);

    std::vector<Point> points;
    double min_x;
    double max_x;
};

}
