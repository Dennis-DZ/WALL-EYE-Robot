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
const double distanceBetweenWheels = 7.25;
const double turnComp = 0.75;
const int pulsePercent = 20;
const double pulseTime = 0.07;
const double rpsWaitTime = 0.5;
const double pi = 3.14159;
const int speed = 6;
const double sleepTime = 0.5;

AnalogInputPin cdsCell(FEHIO::P3_7);

Motor rightMotor(FEHMotor::Motor3, 9.0, distancePerCount, FEHIO::P0_0);
Motor leftMotor(FEHMotor::Motor0, 9.0, distancePerCount, FEHIO::P0_7);

// spatula straight up degree = 150, all the way back = 172, down to ground is 35
Servo spatula(FEHServo::Servo0, 692, 2332);
// gate up degree = 170, open = 70;
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
    double distance = (pi * distanceBetweenWheels) * (degrees / 360.0) * turnComp;
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

    spatula.setDegree(150);

    LCD.Clear(BLACK);
    LCD.SetFontColor(WHITE);

    LCD.ClearBuffer();
    LCD.WriteLine("Tap to load luggage");
    while (!LCD.Touch(&x,&y));
    while (LCD.Touch(&x,&y));

    luggageGate.setDegree(170);
    Sleep(1.0);

    LCD.ClearBuffer();
    LCD.WriteLine("Tap to move back spatula");
    while (!LCD.Touch(&x,&y));
    while (LCD.Touch(&x,&y));

    spatula.setDegree(172);

    RPS.InitializeTouchMenu();

    LCD.ClearBuffer();
    LCD.WriteLine("Tap to start");
    while (!LCD.Touch(&x,&y));
    while (LCD.Touch(&x,&y));

    while (getLightColor() == BLACK);

    spatula.setDegree(150); // move spatula straight up
    Sleep(sleepTime);

    driveToPoint(31, 18, speed, false); // drive to base of ramp
    Sleep(sleepTime);

    turnAndCorrect(90, speed); // turn to top of ramp
    Sleep(sleepTime);

    drive(24, speed); // drive to top of ramp
    Sleep(sleepTime);

    spatula.moveToDegree(35, 0.5); // lower spatula
    Sleep(sleepTime);

    driveToPoint(29.5, 44, speed, false); // line up with stamp
    Sleep(sleepTime);

    turnAndCorrect(90, speed); // face stamp
    Sleep(sleepTime);

    driveToPoint(RPS.X(), 46, speed, false); // drive up to stamp
    Sleep(sleepTime);

    spatula.setDegree(150); // flip up stamp
    Sleep(sleepTime);

    drive(-3, speed); // back up
    Sleep(sleepTime);

    driveToPoint(19, 44.6, speed, true); // drive next to drop off
    Sleep(sleepTime);

    turnAndCorrect(180, speed); // align with drop off
    Sleep(sleepTime);

    luggageGate.setDegree(70); // lower luggage gate
    Sleep(2.0);

    luggageGate.setDegree(170); // raise luggage gate
    Sleep(sleepTime);

    driveToPoint(12.6, 62.3, speed, false); // drive to kiosk light
    Sleep(sleepTime);

    drive(-3, speed); // back up
    Sleep(sleepTime);

    if (getLightColor() == RED) {
        driveToPoint(19.7, 55.7, speed, false); // line up with red button
    } else {
        driveToPoint(14.5, 58.2, speed, false); // line up with blue button
    }

    turnAndCorrect(90, speed); // face kiosk
    Sleep(sleepTime);

    driveForwardUntilStopped(); // drive into kiosk button
    Sleep(sleepTime);

    drive(-5, speed); // back up
    Sleep(sleepTime);

    driveToPoint(5, 45, speed, false); // drive to top of steep ramp
    Sleep(sleepTime);

    turnAndCorrect(270, speed); // turn towards bottom of ramp
    Sleep(sleepTime);

    drive(20, speed); // drive to bottom of ramp
    Sleep(sleepTime);

    switch (RPS.GetCorrectLever()) {
    case 0:
        driveToPoint(7.1, 23, speed, false); // drive up to left lever
        correctHeading(298);
        break;
    case 1:
        driveToPoint(6.8, 24.5, speed, false); // drive up to middle lever
        correctHeading(277);
        break;
    case 2:
    default:
        driveToPoint(7.2, 24.3, speed, false); // drive up to right lever
        correctHeading(257);
        break;
    }

    spatula.setDegree(35); // flip down lever
    Sleep(sleepTime);

    Sleep(5.0); // wait 5 seconds

    spatula.setDegree(150); // flip up lever
    Sleep(sleepTime);

    driveToPoint(36, 0, speed, false); // drive to and press final button
    Sleep(sleepTime);

    return 0;
}
