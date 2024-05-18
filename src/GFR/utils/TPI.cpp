#include "GFR/utils/TPI.hpp"

#include <cmath>

TPI::TPI(double leftTPI, double rightTPI, double trackWidth)
    : leftTPI(leftTPI), rightTPI(rightTPI), trackWidth(trackWidth) {}

TPI::TPI(double tpi, double wheelDiameter) {
    initializeFromTPI(tpi, wheelDiameter);
}

int TPI::inchesToTicks(double inches, bool isLeftEncoder) const {
    return static_cast<int>(inches * (isLeftEncoder ? leftTPI : rightTPI));
}

double TPI::ticksToInches(int ticks, bool isLeftEncoder) const {
    return ticks / (isLeftEncoder ? leftTPI : rightTPI);
}

int TPI::calculateTicksPerRevolution(double tpi, double wheelDiameter) {
    double circumference = wheelDiameter * M_PI;
    return static_cast<int>(tpi * circumference);
}

double TPI::getLeftTPI() const {
    return leftTPI;
}

double TPI::getRightTPI() const {
    return rightTPI;
}

double TPI::getTrackWidth() const {
    return trackWidth;
}

void TPI::initializeFromTPI(double tpi, double wheelDiameter) {
    leftTPI = tpi;
    rightTPI = tpi;
    trackWidth = wheelDiameter * M_PI / tpi;
}
