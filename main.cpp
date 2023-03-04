#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHRPS.h>
#include <string>
#include "Motor.h"
#include <cmath>

const double distancePerCount = 6.0/242;
const double adjustmentFactor = 1.03;
const double distanceBetweenWheels = 7.25;

AnalogInputPin cdsCell(FEHIO::P3_7);

Motor rightMotor(FEHMotor::Motor3, 9.0, distancePerCount, FEHIO::P0_0);
Motor leftMotor(FEHMotor::Motor0, 9.0, distancePerCount, FEHIO::P0_7);

void move(double rightDistance, double leftDistance, double time) {

    const double sleepTime = 0.15;
    const int rightSign = (rightDistance > 0) - (rightDistance < 0), leftSign = (leftDistance > 0) - (leftDistance < 0);

    double counts = (std::abs(rightDistance) + std::abs(leftDistance)) * (1 / distancePerCount) * adjustmentFactor;
    double rightSpeed = rightDistance / time, leftSpeed = leftDistance / time;
    double t = TimeNow();
    int lastRightCounts = 0, lastLeftCounts = 0;

    rightMotor.reset();
    leftMotor.reset();
    
    Sleep(sleepTime);

    while (leftMotor.counts() + rightMotor.counts() < counts) {
        rightMotor.setPercent(rightSign * rightMotor.PIDAdjustment(rightMotor.counts() - lastRightCounts, TimeNow() - t, rightSpeed));
        leftMotor.setPercent(leftSign * leftMotor.PIDAdjustment(leftMotor.counts() - lastLeftCounts, TimeNow() - t, leftSpeed));
        t = TimeNow();
        lastRightCounts = rightMotor.counts();
        lastLeftCounts = leftMotor.counts();
        Sleep(sleepTime);
        if (rightMotor.counts() == lastRightCounts && leftMotor.counts() == lastLeftCounts) {
            break;
        }
    }

    rightMotor.stop();
    leftMotor.stop();
}

void drive(double distance, double speed) {
    move(distance, distance, distance/speed);
    std::string message = "Drove " + std::to_string(distance) + " in";
    LCD.WriteLine(message.c_str());
}

void turn(int degrees, double speed) {
    double distance = (3.14159 * distanceBetweenWheels) * (degrees / 360.0);
    move(-1 * distance, distance, distance/speed);
    std::string message = "Turned " + std::to_string(degrees) + " degrees";
    LCD.WriteLine(message.c_str());
}

int main() {

    const int speed = 6;
    const double sleepTime = 0.5;

    float x, y;

    LCD.Clear(BLACK);
    LCD.SetFontColor(WHITE);

    LCD.WriteLine("Touch the screen");
    while (!LCD.Touch(&x,&y));
    while (LCD.Touch(&x,&y));

    // while (cdsCell.Value() > 2) {
    //     LCD.WriteLine(cdsCell.Value());
    // }

    turn(4, speed);
    Sleep(sleepTime);

    drive(35, speed);
    Sleep(sleepTime);

    turn(-94, speed);
    Sleep(sleepTime);

    drive(10, speed);
    Sleep(sleepTime);

    turn(90, speed);
    Sleep(sleepTime);

    drive(20, speed);
    Sleep(sleepTime);

    drive(-19, speed);
    Sleep(sleepTime);

    turn(-90, speed);
    Sleep(sleepTime);

    drive(-10, speed);
    Sleep(sleepTime);

    turn(94, speed);
    Sleep(sleepTime);

    drive(35, speed);
    Sleep(sleepTime);

    return 0;
}
