#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHRPS.h>
#include <string>
#include "Motor.h"
#include "Servo.h"
#include <cmath>
#include <FEHServo.h>

using namespace std;

const double distancePerCount = 6.0/242;
//const double adjustmentFactor = 1.03;
const double distanceBetweenWheels = 7.25;
const double turnComp = 0.98;
const int pulsePercent = 15;
const double pulseTime = 0.1;
const double pi = 3.14159;

AnalogInputPin cdsCell(FEHIO::P3_7);

Motor rightMotor(FEHMotor::Motor3, 9.0, distancePerCount, FEHIO::P0_0);
Motor leftMotor(FEHMotor::Motor0, 9.0, distancePerCount, FEHIO::P0_7);

Servo spatula(FEHServo::Servo0, 692, 2332);

enum Direction {
    CW,
    CCW
};

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
    double distance = (pi * distanceBetweenWheels) * (degrees / 360.0);
    move(-1 * distance, distance, distance/speed);
    string message = "Turned " + to_string(degrees) + " degrees";
    LCD.WriteLine(message.c_str());
}

int getLightColor() {
    double reading = cdsCell.Value();
    //LCD.WriteLine(reading);
    if (reading < 0.55) {
        //LCD.WriteLine("Light Color: RED");
        return RED;
    } else if (reading < 2) {
        //LCD.WriteLine("Light Color: BLUE");
        return BLUE;
    } else {
        //LCD.WriteLine("Light Color: NONE");
        return BLACK;
    }
}

void pulseTurn(Direction d) {
    rightMotor.setPercent(pulsePercent * (d == CCW));
    leftMotor.setPercent(pulsePercent * (d == CW));
    Sleep(pulseTime);
    rightMotor.stop();
    leftMotor.stop();
}

void correctHeading(int degrees, Direction d) {
    while(RPS.Heading() < degrees - 1 || RPS.Heading() > degrees + 1) {
        pulseTurn(d);
        while (RPS.Heading() < 0);
    }
}

void correctHeading(double x, double y, Direction d) {
    while (RPS.Heading() < 0);
    double deltaX = x - RPS.X(), deltaY = y - RPS.Y();
    correctHeading(270 + atan(deltaY/deltaX) * (180 / pi), d);
}

void driveForwardUntilStopped() {
    rightMotor.setPercent(25);
    leftMotor.setPercent(25);
    int lastCounts = 0;
    Sleep(0.25);
    while (rightMotor.counts() - lastCounts > 10) {
        lastCounts = rightMotor.counts();
        Sleep(0.25);
    }
}

double checkAndSleep(double minimum) {
    Sleep(0.5);
    if (cdsCell.Value() < minimum) {
        return cdsCell.Value();
    } else {
        return minimum;
    }
}

int moveAround(double speed) {
    double minimum = 3.3;

    double distance = 0.5;

    drive(-1 * distance, speed);
    minimum = checkAndSleep(minimum);

    drive(-1 * distance, speed);
    minimum = checkAndSleep(minimum);

    turn(10, speed);
    minimum = checkAndSleep(minimum);

    drive(distance, speed);
    minimum = checkAndSleep(minimum);

    drive(distance, speed);
    minimum = checkAndSleep(minimum);

    drive(-1 * distance, speed);
    minimum = checkAndSleep(minimum);

    drive(-1 * distance, speed);
    minimum = checkAndSleep(minimum);

    turn(-20, speed);
    minimum = checkAndSleep(minimum);

    drive(distance, speed);
    minimum = checkAndSleep(minimum);

    drive(distance, speed);
    minimum = checkAndSleep(minimum);

    drive(-1 * distance, speed);
    minimum = checkAndSleep(minimum);

    drive(-1 * distance, speed);
    minimum = checkAndSleep(minimum);

    turn(10, speed);
    minimum = checkAndSleep(minimum);

    drive(distance, speed);
    minimum = checkAndSleep(minimum);

    drive(distance, speed);
    minimum = checkAndSleep(minimum);

    if (minimum < 0.55) {
        //LCD.WriteLine("Light Color: RED");
        return RED;
    } else if (minimum < 2) {
        //LCD.WriteLine("Light Color: BLUE");
        return BLUE;
    } else {
        //LCD.WriteLine("Light Color: NONE");
        return BLACK;
    }
}

int main() {

    const int speed = 6;
    const double sleepTime = 0.5;

    float x, y;

    // RPS.InitializeTouchMenu();

    spatula.setDegree(180);

    LCD.Clear(BLACK);
    LCD.SetFontColor(WHITE);

    LCD.WriteLine("Touch the screen");
    while (!LCD.Touch(&x,&y));
    while (LCD.Touch(&x,&y));

    while (getLightColor() == BLACK);

    int correctLever = /*RPS.GetCorrectLever()*/0;

    drive(10, speed);
    Sleep(sleepTime);

    turn(-75, speed);
    Sleep(sleepTime);

    drive(14, speed);
    Sleep(sleepTime);

    turn(-5, speed);
    Sleep(sleepTime);

    if (correctLever == 1) {

        drive(3.5, speed);
        Sleep(sleepTime);

    } else if (correctLever == 2) {

        drive(7, speed);
        Sleep(sleepTime);
        
    }

    turn(-76, speed);
    Sleep(sleepTime);

    drive(-1.4, speed);
    Sleep(sleepTime);

    // while (true) {

        spatula.moveToDegree(60, 0.5);

        Sleep(5.0);

        spatula.moveToDegree(180, 0.5);

    //     while (!LCD.Touch(&x,&y));
    //     // while (LCD.Touch(&x,&y));

    // }

    return 0;
}
