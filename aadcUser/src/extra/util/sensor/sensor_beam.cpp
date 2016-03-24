#include "sensor_beam.h"

#include <math.h>
#include <iostream>

// -------------------------------------------------------------------------------------------------
SensorBeam::SensorBeam(Vector2 const & origin, int angle, double length) : origin_(origin) {
// -------------------------------------------------------------------------------------------------
    angle_ = angle;
    length_ = length;
}

// -------------------------------------------------------------------------------------------------
Vector2 SensorBeam::getEnd() const {
// -------------------------------------------------------------------------------------------------
    double radian = angle_ * M_PI / 180.0;
    int x_end_norm = origin_.get_x() + (cos(radian) * length_);
    int y_end_norm = origin_.get_y() + (sin(radian) * length_);

    return Vector2(x_end_norm, y_end_norm);
}

// -------------------------------------------------------------------------------------------------
std::vector<Vector2> SensorBeam::getLinePoints() const {
// -------------------------------------------------------------------------------------------------
    std::vector<Vector2> result;

    Vector2 end = getEnd();

    int x0 = origin_.get_x();
    int x1 = end.get_x();
    int y0 = origin_.get_y();
    int y1 = end.get_y();
    int dX = fabs(x1 - x0);
    int dY = - fabs(y1 - y0);
    int slX = (x0 < x1) ? 1 : -1;
    int slY = (y0 < y1) ? 1 : -1;
    int err = dX + dY;

    while (true) {
        result.push_back(Vector2(x0, y0));

        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > dY) {
            err += dY;
            x0 += slX;
        }

        if (e2 < dX) {
            err += dX;
            y0 += slY;
        }

        // Do not add the end point to the line
        if (end.get_x() == x0 && end.get_y() == y0) break;
    }

    return result;
}