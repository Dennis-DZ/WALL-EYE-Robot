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
const int speed = 10;
const double sleepTime = 0.25;

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

void println() {
    SD.FPrintf(outFile, "\n");
}

void log(string message) {
    LCD.WriteLine(message.c_str());
    SD.FPrintf(outFile, (message + "\n").c_str());
}

void move(double rightDistance, double leftDistance, double time) {

    const double pidSleepTime = 0.15;
    int rightSign = (rightDistance > 0) - (rightDistance < 0), leftSign = (leftDistance > 0) - (leftDistance < 0);
    double counts = (abs(rightDistance) + abs(leftDistance)) * (1 / distancePerCount);
    double rightSpeed = rightDistance / time, leftSpeed = leftDistance / time;
    double t = TimeNow();

    rightMotor.reset();
    leftMotor.reset();
    
    Sleep(pidSleepTime);

    while (leftMotor.counts() + rightMotor.counts() < counts) {
        rightMotor.setPercent(rightSign * rightMotor.PIDAdjustment(TimeNow() - t, rightSpeed));
        leftMotor.setPercent(leftSign * leftMotor.PIDAdjustment(TimeNow() - t, leftSpeed));
        t = TimeNow();
        Sleep(pidSleepTime);
    }

    rightMotor.stop();
    leftMotor.stop();
}

void drive(double distance, double speed) {
    move(distance, distance, distance/speed);
    Sleep(rpsWaitTime);
}

void turn(int degrees, double speed) {
    double distance = (pi * distanceBetweenWheels) * (degrees / 360.0) * turnComp;
    move(-1 * distance, distance, distance/speed);
}

int getLightColor(double reading) {
    if (reading < 0.55) {
        log("Light Color: RED");
        return RED;
    } else if (reading < 2) {
        log("Light Color: BLUE");
        return BLUE;
    } else {
        return BLACK;
    }
}

double getMinCdsReading() {

    double rightWheel[8] = {-1, 0, -1, 2, 2, -2, -3, 2};
    double leftWheel[8] = {0, -1, -1, 0, 2, -2, 3, 2};
    double min = 3.3;

    for (int i = 0; i < 8; i++) {
        move(rightWheel[i], leftWheel[i], 0.1 * max(abs(rightWheel[i]), abs(leftWheel[i])));
        double reading = cdsCell.Value();
        getLightColor(reading);
        log(reading);
        if (reading < min) {
            min = reading;
        }
    }

    return min;
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

void pulseForward() {
    rightMotor.setPercent(pulsePercent);
    leftMotor.setPercent(pulsePercent);
    Sleep(pulseTime*2);
    rightMotor.stop();
    leftMotor.stop();
}

void pulseBackward() {
    rightMotor.setPercent(-1 * pulsePercent);
    leftMotor.setPercent(-1 * pulsePercent);
    Sleep(pulseTime*2);
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

    while (RPS.Heading() < degrees - 2 || RPS.Heading() > degrees + 2) {
        pulseTurn(directionOfTurn(degrees - RPS.Heading()));
        Sleep(rpsWaitTime);
        while (RPS.Heading() < 0);
    }
    println();
    log("Corrected heading to " + to_string(RPS.Heading()));
}

void checkY(double y, bool backwards) {
    while (RPS.Y() < y - 0.25 || RPS.Y() > y + 0.25) {
        if ((backwards && RPS.Y() > y) || (!backwards && RPS.Y() < y)) {
            pulseForward();
        } else {
            pulseBackward();
        }
        Sleep(rpsWaitTime);
        while (RPS.Heading() < 0);
    }
    println();
    log("Corrected Y to " + to_string(RPS.Y()));
}

void checkX(double x, bool backwards) {
    while (RPS.X() < x - 0.25 || RPS.X() > x + 0.25) {
        if ((backwards && RPS.X() > x) || (!backwards && RPS.X() < x)) {
            pulseForward();
        } else {
            pulseBackward();
        }
        Sleep(rpsWaitTime);
        while (RPS.Heading() < 0);
    }
    println();
    log("Corrected X to " + to_string(RPS.X()));
}

double distanceBetween(int currentX, int currentY, int targetX, int targetY) {
    int deltaX = targetX - currentX, deltaY = targetY - currentY;
    return sqrt(deltaX * deltaX + deltaY * deltaY);
}

void turnAndCorrect(int degrees, double speed) {
    println();
    log("Turning to " + to_string(degrees) + " from " + to_string(RPS.Heading()));

    double divisor = 1;
    while (RPS.Heading() < 0);
    Sleep(rpsWaitTime);
    while (abs(signedDegreeDifference(RPS.Heading(), degrees)) > 10) {
        turn(signedDegreeDifference(RPS.Heading(), degrees) / divisor++, speed);
        Sleep(rpsWaitTime);
        log("Facing " + to_string(RPS.Heading()));
    }
    correctHeading(degrees);
}

void driveToPoint(double x, double y, double speed, bool careful) {
    println();
    log("Driving to " + to_string(x) + ", " + to_string(y) + " from " + to_string(RPS.X()) + ", " + to_string(RPS.Y()));

    double factor = 1 - 0.34 * careful;
    turnAndCorrect(getHeadingToPoint(x, y), speed);

    while(RPS.X() < 0);
    double distance = distanceBetween(RPS.X(), RPS.Y(), x, y);

    while (distance > 15 * factor) {
        correctHeading(getHeadingToPoint(x, y));
        drive(7 * factor, speed);
        //Sleep(rpsWaitTime);
        while(RPS.X() < 0);
        distance = distanceBetween(RPS.X(), RPS.Y(), x, y);
    }

    correctHeading(getHeadingToPoint(x, y));
    drive(distance, speed);
    //Sleep(rpsWaitTime);

    println();
    log("Drove to " + to_string(RPS.X()) + ", " + to_string(RPS.Y()));
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

    RPS.InitializeTouchMenu();
    spatula.setDegree(172);

    LCD.ClearBuffer();
    LCD.WriteLine("Tap to start");
    while (!LCD.Touch(&x,&y));
    while (LCD.Touch(&x,&y));

    LCD.WriteLine("Waiting for light");

    while (getLightColor(cdsCell.Value()) == BLACK); // wait for start light

    spatula.setDegree(150); // move spatula straight up

    driveToPoint(32, 18, speed, false); // drive to base of ramp

    turnAndCorrect(90, speed); // turn to top of ramp

    drive(20, speed); // drive to top of ramp

    checkY(40.6, false); // adjust y

    spatula.moveToDegree(35, 0.5); // lower spatula

    driveToPoint(29.5, 44, speed, false); // line up with stamp

    turnAndCorrect(90, speed); // face stamp

    driveToPoint(RPS.X(), 45.75, speed, false); // drive up to stamp

    checkY(47, false); // adjust y

    spatula.setDegree(150); // flip up stamp

    drive(-3, speed); // back up

    driveToPoint(19, 44.6, speed, true); // drive next to drop off

    turnAndCorrect(180, speed); // align with drop off

    checkX(18, true); // adjust x

    luggageGate.setDegree(70); // lower luggage gate

    Sleep(1.0); // wait for luggage to fall

    luggageGate.setDegree(170); // raise luggage gate

    driveToPoint(12.6, 61.4, speed, false); // drive to kiosk light

    int lightColor = getLightColor(getMinCdsReading()); // move around to get best light reading

    turnAndCorrect(135, speed); // turn to face wall

    drive(-10, speed); // back up

    if (lightColor == RED) {
        driveToPoint(23.5, 58.8, speed, false); // line up with red button
    } else {
        driveToPoint(17.6, 58.5, speed, false); // line up with blue button
    }

    turnAndCorrect(90, speed); // face kiosk

    driveForwardUntilStopped(); // drive into kiosk button

    drive(-5, speed); // back up

    driveToPoint(5, 45, speed, false); // drive to top of steep ramp

    turnAndCorrect(270, speed); // turn towards bottom of ramp

    drive(15, speed); // drive to bottom of ramp

    checkY(25.5, true); // adjust y

    switch (RPS.GetCorrectLever()) {
    case 0:
        driveToPoint(7.1, 23, speed, false); // drive up to left lever
        correctHeading(298);
        break;
    case 1:
        driveToPoint(5.7, 23.4, speed, false); // drive up to middle lever
        correctHeading(287);
        drive(-0.15, speed);
        break;
    case 2:
    default:
        driveToPoint(5.1, 24, speed, false); // drive up to right lever
        correctHeading(269);
        break;
    }

    spatula.setDegree(35); // flip down lever

    Sleep(5.5); // wait 5.5 seconds

    spatula.setDegree(150); // flip up lever

    driveToPoint(17.4, 22.7, speed, false); // drive towards final button avoiding dead zone

    driveToPoint(36, 0, speed, false); // drive to and press final button

    return 0;
}
