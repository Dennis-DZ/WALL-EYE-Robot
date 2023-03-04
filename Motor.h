#include <FEHMotor.h>
#include <FEHIO.h>
#include <cmath>

class Motor {

private:

    FEHMotor motor = FEHMotor(FEHMotor::Motor0, 0);
    DigitalEncoder encoder = DigitalEncoder(FEHIO::P0_0);
    double errorSum, lastError, power;
    double distancePerCount;

public:

    Motor(FEHMotor::FEHMotorPort motorPort, double voltage, double dpc, FEHIO::FEHIOPin encoderPin) {
        motor = FEHMotor(motorPort, voltage);
        encoder = DigitalEncoder(encoderPin);
        distancePerCount = dpc;
        errorSum = 0;
        power = 0;
        lastError = 0;
        encoder.ResetCounts();
    }

    void reset() {
        errorSum = 0;
        power = 0;
        lastError = 0;
        encoder.ResetCounts();
    }

    void setPercent(double percent) {
        motor.SetPercent(percent);
    }

    double PIDAdjustment(int deltaCounts, double deltaTime, double speed) {
        const double P = 0.9, I = 0.09, D = 0.3;
        double error = std::abs(speed) - (distancePerCount * deltaCounts) / deltaTime;
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