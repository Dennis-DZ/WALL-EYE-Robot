#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHRPS.h>
#include <string>
#include "Motor.h"
#include <cmath>

using namespace std;

const double distancePerCount = 6.0/242;
const double adjustmentFactor = 1.03;
const double distanceBetweenWheels = 7.25;
const double distanceComp = 0.97;

AnalogInputPin cdsCell(FEHIO::P3_7);

Motor rightMotor(FEHMotor::Motor3, 9.0, distancePerCount, FEHIO::P0_0);
Motor leftMotor(FEHMotor::Motor0, 9.0, distancePerCount, FEHIO::P0_7);

void move(double rightDistance, double leftDistance, double time) {

    double sleepTime = 0.15;
    int rightSign = (rightDistance > 0) - (rightDistance < 0), leftSign = (leftDistance > 0) - (leftDistance < 0);
    double counts = (abs(rightDistance) + abs(leftDistance)) * (1 / distancePerCount);
    double rightSpeed = rightDistance / time, leftSpeed = leftDistance / time;
    double t = TimeNow();

    rightMotor.reset();
    leftMotor.reset();
    
    Sleep(sleepTime);

    while (leftMotor.counts() + rightMotor.counts() < counts) {
        rightMotor.setPercent(rightSign * rightMotor.PIDAdjustment(TimeNow() - t, rightSpeed));
        leftMotor.setPercent(leftSign * leftMotor.PIDAdjustment(TimeNow() - t, leftSpeed));
        t = TimeNow();
        Sleep(sleepTime);
        if (rightMotor.isStalled() && leftMotor.isStalled()) {
            break;
        }
    }

    rightMotor.stop();
    leftMotor.stop();
}

void drive(double distance, double speed) {
    move(distance, distance, distance/speed);
    string message = "Drove " + to_string(distance) + " in";
    LCD.WriteLine(message.c_str());
}

void turn(int degrees, double speed) {
    double distance = (3.14159 * distanceBetweenWheels) * (degrees / 360.0);
    move(-1 * distance, distance, distance/speed);
    string message = "Turned " + to_string(degrees) + " degrees";
    LCD.WriteLine(message.c_str());
}

int getLightColor() {
    double reading = cdsCell.Value();
    LCD.WriteLine(reading);
    if (reading < 0.4) {
        LCD.WriteLine("Light Color: RED");
        return RED;
    } else if (reading < 1) {
        LCD.WriteLine("Light Color: BLUE");
        return BLUE;
    } else {
        LCD.WriteLine("Light Color: NONE");
        return BLACK;
    }
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

    while (getLightColor() == BLACK);

    turn(2, speed);
    Sleep(sleepTime);

    drive(34.5, speed);
    Sleep(sleepTime);

    turn(-40, speed);
    Sleep(sleepTime);

    drive(25, speed);
    Sleep(sleepTime);

    int color = getLightColor();
    Sleep(sleepTime);

    if (color == BLUE) {
        drive(-5, speed);
    } else if (color == RED) {
        drive(-14, speed);
    } else {
        return 0;
    }

    turn(40, speed);
    Sleep(sleepTime);

    // move until stopped
    rightMotor.setPercent(25);
    leftMotor.setPercent(25);
    int lastCounts = 0;
    Sleep(0.25);
    while (rightMotor.counts() - lastCounts > 10) {
        lastCounts = rightMotor.counts();
        Sleep(0.25);
    }

    drive(-18, speed);
    Sleep(sleepTime);

    turn(-90, speed);
    Sleep(sleepTime);

    if (color == BLUE) {
        drive(-13, speed);
    } else if (color == RED) {
        drive(-6, speed);
    }

    turn(90, speed);
    Sleep(sleepTime);

    drive(-25, speed);
    Sleep(sleepTime);

    return 0;
}
