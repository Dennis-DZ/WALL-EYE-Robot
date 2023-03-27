#include <FEHServo.h>
#include <cmath>
#include <FEHUtility.h>

class Servo {

private:

    FEHServo servo = FEHServo(FEHServo::Servo0);
    double currentDegree;

public:

    Servo(FEHServo::FEHServoPort servoPort, int min, int max) {
        servo = FEHServo(servoPort);
        servo.SetMin(min);
        servo.SetMax(max);
        currentDegree = 0;
    }

    void setDegree(double degree) {
        servo.SetDegree(degree);
        currentDegree = degree;
    }

    void moveToDegree(int degree, double seconds) {
        double degreesPerSecond = (degree - currentDegree) / seconds;
        double t = TimeNow();

        while (abs(degree - currentDegree) > 1) {
            setDegree(currentDegree + degreesPerSecond * (TimeNow() - t));
            LCD.WriteLine(currentDegree);
            t = TimeNow();
            Sleep(0.05);
        }
    }

    void touchCalibrate() {
        servo.TouchCalibrate();
    }

};
