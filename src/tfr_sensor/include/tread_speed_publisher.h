
class TreadSpeed {
public:
    double speed;

    TreadSpeed(const int ticksPerRevolution, const int maxTicks, const double wheelRadius, const int prevTickCount = 0);

    void updateFromNewCount(const int newCount);
    
private:

    int calcTickDiff(const int newCount);

    int prevTickCount; // last recorded position of wheel
    const int ticksPerRevolution; // number of ticks counted each revolution of the measured wheel
    const int maxTicks; // number of ticks counted before rolling over back to 0
    const double wheelRadius; // radius of wheel (for which ticks are being counted) in meters
};
