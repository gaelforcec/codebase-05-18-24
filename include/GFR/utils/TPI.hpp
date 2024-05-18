#ifndef TPI_HPP
#define TPI_HPP

class TPI {
public:
    TPI(double leftTPI, double rightTPI, double trackWidth);
    TPI(double tpi, double wheelDiameter);
    int inchesToTicks(double inches, bool isLeftEncoder) const;
    double ticksToInches(int ticks, bool isLeftEncoder) const;
    static int calculateTicksPerRevolution(double tpi, double wheelDiameter);
    double getLeftTPI() const;
    double getRightTPI() const;
    double getTrackWidth() const;

private:
    double leftTPI;
    double rightTPI;
    double trackWidth;
    void initializeFromTPI(double tpi, double wheelDiameter);
};

#endif 
