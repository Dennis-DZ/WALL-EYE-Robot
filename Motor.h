#include <FEHMotor.h>
#include <FEHIO.h>
#include <cmath>

class Motor {

private:

    FEHMotor motor = FEHMotor(FEHMotor::Motor0, 0);
    DigitalEncoder encoder = DigitalEncoder(FEHIO::P0_0);
    double errorSum, lastError, power;
    int lastCounts;
    double distancePerCount;

public:

    Motor(FEHMotor::FEHMotorPort motorPort, double voltage, double dpc, FEHIO::FEHIOPin encoderPin) {
        motor = FEHMotor(motorPort, voltage);
        encoder = DigitalEncoder(encoderPin);
        distancePerCount = dpc;
        reset();
    }

    void reset() {
        errorSum = 0;
        power = 0;
        lastError = 0;
        lastCounts = 0;
        encoder.ResetCounts();
    }

    void setPercent(double percent) {
        motor.SetPercent(percent);
        lastCounts = counts();
    }

    double PIDAdjustment(double deltaTime, double speed) {
        const double P = 0.9, I = 0.09, D = 0.3;
        double error = std::abs(speed) - (distancePerCount * (counts() - lastCounts)) / deltaTime;
        errorSum += error;
        power += (error * P) + (errorSum * I) + ((error - lastError) * D);
        lastError = error;
        return power;
    }

    void stop() {
        motor.Stop();
    }

    int counts() {
        return encoder.Counts();
    }

};
