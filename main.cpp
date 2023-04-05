#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHRPS.h>
#include <string>
#include "Motor.h"
#include "Servo.h"
#include <cmath>
#include <FEHServo.h>
#include <FEHSD.h>

using namespace std;

const double distancePerCount = 6.0/242;
//const double adjustmentFactor = 1.03;
const double distanceBetweenWheels = 7.25;
// const double turnComp = 0.98;
const int pulsePercent = 20;
const double pulseTime = 0.07;
const double rpsWaitTime = 0.5;
const double pi = 3.14159;
const int speed = 6;
const double sleepTime = 0.5;

AnalogInputPin cdsCell(FEHIO::P3_7);

Motor rightMotor(FEHMotor::Motor3, 9.0, distancePerCount, FEHIO::P0_0);
Motor leftMotor(FEHMotor::Motor0, 9.0, distancePerCount, FEHIO::P0_7);

Servo spatula(FEHServo::Servo0, 692, 2332);
Servo luggageGate(FEHServo::Servo1, 500, 2415);

FEHFile *outFile;

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
    SD.FPrintf(outFile, (message + "\n").c_str());
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

int getHeadingToPoint(double x, double y) {
    while (RPS.Heading() < 0);
    double deltaX = x - RPS.X(), deltaY = y - RPS.Y();
    double arctan = atan(deltaY/deltaX) * (180 / pi);

	if (deltaX < 0) {
		return 180 + arctan;
	} else if (arctan >= 0) {
		return arctan;
	} else {
		return 360 + arctan;
	}
}

void pulseTurn(Direction d) {
    rightMotor.setPercent(pulsePercent * (-1 + 2 * (d == CCW)));
    leftMotor.setPercent(pulsePercent * (-1 + 2 * (d == CW)));
    Sleep(pulseTime);
    rightMotor.stop();
    leftMotor.stop();
}

Direction directionOfTurn(int delta) {
    return (delta >= 0 && delta <= 180) || (delta <= -180 && delta >= -360) ? CCW : CW;
}

int signedDegreeDifference(int current, int target) {

    // https://stackoverflow.com/a/30887154

    int delta = target - current;

    int d = abs(delta) % 360; 
    int r = d > 180 ? 360 - d : d;

    if (directionOfTurn(delta) == CCW) {
        r *= -1;
    }

    return r;
}

void correctHeading(int degrees) {
    while(RPS.Heading() < degrees - 1 || RPS.Heading() > degrees + 1) {
        pulseTurn(directionOfTurn(degrees - RPS.Heading()));
        Sleep(rpsWaitTime);
        while (RPS.Heading() < 0);
    }

    string message = "Now facing " + to_string(RPS.Heading());
    LCD.WriteLine(message.c_str());
}

double distanceBetween(int currentX, int currentY, int targetX, int targetY) {
    int deltaX = targetX - currentX, deltaY = targetY - currentY;
    return sqrt(deltaX * deltaX + deltaY * deltaY);
}

void turnAndCorrect(int degrees, double speed) {
    while (RPS.Heading() < 0);
    turn(signedDegreeDifference(RPS.Heading(), degrees), speed);
    correctHeading(degrees);
}

void driveToPoint(double x, double y, double speed, bool careful) {

    double factor = 1 - 0.34 * careful;

    turnAndCorrect(getHeadingToPoint(x, y), speed);

    while(RPS.X() < 0);
    double distance = distanceBetween(RPS.X(), RPS.Y(), x, y);

    while (distance > 15 * factor) {
        correctHeading(getHeadingToPoint(x, y));
        drive(7 * factor, speed);
        Sleep(rpsWaitTime);
        while(RPS.X() < 0);
        distance = distanceBetween(RPS.X(), RPS.Y(), x, y);
    }

    correctHeading(getHeadingToPoint(x, y));
    drive(distance, speed);
    Sleep(rpsWaitTime);

    string message = "Drove to " + to_string(RPS.X()) + ", " + to_string(RPS.Y());
    LCD.WriteLine(message.c_str());
    SD.FPrintf(outFile, (message + "\n").c_str());
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

int main() {

    outFile = SD.FOpen("log.txt", "w");

    float x, y;

    spatula.setDegree(180);

    LCD.Clear(BLACK);
    LCD.SetFontColor(WHITE);

    LCD.ClearBuffer();
    LCD.WriteLine("Touch the screen to load luggage");
    while (!LCD.Touch(&x,&y));
    while (LCD.Touch(&x,&y));

    luggageGate.setDegree(170);

    RPS.InitializeTouchMenu();

    LCD.ClearBuffer();
    LCD.WriteLine("Touch the screen to start");
    while (!LCD.Touch(&x,&y));
    while (LCD.Touch(&x,&y));

    while (getLightColor() == BLACK);

    driveToPoint(31, 18, speed, false); // drive to base of ramp
    Sleep(sleepTime);

    turnAndCorrect(90, speed); // turn to top of ramp
    Sleep(sleepTime);

    drive(26, speed); // drive to top of ramp
    Sleep(sleepTime);

    driveToPoint(19, 44.6, speed, true); // drive next to drop off
    Sleep(sleepTime);

    turnAndCorrect(180, speed); // align with drop off
    Sleep(sleepTime);

    luggageGate.setDegree(70); // lower luggage gate
    Sleep(2.0);

    luggageGate.setDegree(170); // raise luggage gate
    Sleep(sleepTime);

    driveToPoint(31, 42, speed, true); // drive to top of ramp
    Sleep(sleepTime);

    turnAndCorrect(270, speed); // turn to bottom of ramp
    Sleep(sleepTime);

    drive(26, speed); // drive to bottom of ramp
    Sleep(sleepTime);

    turnAndCorrect(255, speed); // turn towards final button
    Sleep(sleepTime);

    driveForwardUntilStopped(); // run into final button
    Sleep(sleepTime);

    return 0;
}
