#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHRPS.h>
#include <string>

DigitalEncoder rightEncoder(FEHIO::P0_0);
DigitalEncoder leftEncoder(FEHIO::P0_7);
const int countsPer6Inch = 242;
const double distancePerCount = 6.0/242;
const double adjustmentFactor = 1.03;
const double distanceBetweenWheels = 7.25;
const double rightMotorComp = 1.02;
const double rightMotorTurnComp = 0.98;

AnalogInputPin cdsCell(FEHIO::P3_7);

FEHMotor rightMotor(FEHMotor::Motor3, 9.0);
FEHMotor leftMotor(FEHMotor::Motor0, 9.0);

void driveForward(int percent, double distance) {

    double counts = (distance * countsPer6Inch) / 6 * adjustmentFactor;

    rightEncoder.ResetCounts();
    leftEncoder.ResetCounts();

    rightMotor.SetPercent(percent * rightMotorComp);
    leftMotor.SetPercent(percent);

    while ((leftEncoder.Counts() + rightEncoder.Counts()) / 2.0 < counts);

    rightMotor.Stop();
    leftMotor.Stop();

    std::string message = "Drove " + std::to_string(distance) + " in";
    LCD.WriteLine(message.c_str());
}

void driveEqual(int percent, double distance) {

    double counts = (distance * countsPer6Inch) / 6 * adjustmentFactor;

    rightEncoder.ResetCounts();
    leftEncoder.ResetCounts();

    rightMotor.SetPercent(percent);
    leftMotor.SetPercent(percent);

    while ((leftEncoder.Counts() + rightEncoder.Counts()) / 2.0 < counts);

    rightMotor.Stop();
    leftMotor.Stop();

    std::string message = "Drove " + std::to_string(distance) + " in";
    LCD.WriteLine(message.c_str());
}

void driveUntilStopped(int percent, double distance) {

    double counts = (distance * countsPer6Inch) / 6 * adjustmentFactor;

    rightEncoder.ResetCounts();
    leftEncoder.ResetCounts();

    rightMotor.SetPercent(percent * rightMotorComp);
    leftMotor.SetPercent(percent);

    float t = TimeNow();
    int oldCounts;

    while ((leftEncoder.Counts() + rightEncoder.Counts()) / 2.0 < counts) {
        if (TimeNow() > 0.5 + t) {
            t = TimeNow();
            if (leftEncoder.Counts() - oldCounts < 10) {
                break;
            }
            oldCounts = leftEncoder.Counts();
        }
    }

    rightMotor.Stop();
    leftMotor.Stop();

    std::string message = "Drove " + std::to_string(distance) + " in";
    LCD.WriteLine(message.c_str());
}

void backUp(int percent, double distance) {

    double counts = (distance * countsPer6Inch) / 6 * adjustmentFactor;

    rightEncoder.ResetCounts();
    leftEncoder.ResetCounts();

    rightMotor.SetPercent(percent * rightMotorComp *-1);
    leftMotor.SetPercent(percent *-1);

    while ((leftEncoder.Counts() + rightEncoder.Counts()) / 2.0 < counts);

    rightMotor.Stop();
    leftMotor.Stop();

    std::string message = "Backed up " + std::to_string(distance) + " in";
    LCD.WriteLine(message.c_str());
}

void turn(int percent, int degrees) {
    
    double distance = (3.14159 * distanceBetweenWheels) * (std::abs(degrees) / 360.0);
    double counts = (distance * countsPer6Inch) / 6 * adjustmentFactor;
    int leftPercent = percent * (degrees / std::abs(degrees));

    rightEncoder.ResetCounts();
    leftEncoder.ResetCounts();

    rightMotor.SetPercent(leftPercent * -1 * rightMotorTurnComp);
    leftMotor.SetPercent(leftPercent);

    while ((leftEncoder.Counts() + rightEncoder.Counts()) / 2.0 < counts);

    rightMotor.Stop();
    leftMotor.Stop();

    std::string message = "Turned " + std::to_string(degrees) + " degrees";
    LCD.WriteLine(message.c_str());
}

double rightErrorSum = 0, leftErrorSum = 0, lastRightError = 0, lastLeftError = 0;
double rightPower = 0, leftPower = 0;
const double P = 0.9, I = 0.09, D = 0.3;

double rightPIDAdjustment(int deltaCounts, double deltaTime, double speed) {
    double error = speed - (distancePerCount * deltaCounts) / deltaTime;
    rightErrorSum += error;
    rightPower = rightPower + (error * P) + (rightErrorSum * I) + ((error - lastRightError) * D);
    lastRightError = error;
    return rightPower;
}

double leftPIDAdjustment(int deltaCounts, double deltaTime, double speed) {
    double error = speed - (distancePerCount * deltaCounts) / deltaTime;
    leftErrorSum += error;
    leftPower = leftPower + (error * P) + (leftErrorSum * I) + ((error - lastLeftError) * D);
    lastLeftError = error;
    return leftPower;
}

void driveForwardPID(double distance, double speed) {

    const double sleepTime = 0.15;

    double counts = distance * (1 / distancePerCount) * adjustmentFactor;
    double t = TimeNow();
    int lastRightCounts = 0, lastLeftCounts = 0;
    rightEncoder.ResetCounts();
    leftEncoder.ResetCounts();
    
    Sleep(sleepTime);

    while ((leftEncoder.Counts() + rightEncoder.Counts()) / 2.0 < counts) {
        rightMotor.SetPercent(rightPIDAdjustment(rightEncoder.Counts() - lastRightCounts, TimeNow() - t, speed));
        leftMotor.SetPercent(leftPIDAdjustment(leftEncoder.Counts() - lastLeftCounts, TimeNow() - t, speed));
        t = TimeNow();
        lastRightCounts = rightEncoder.Counts();
        lastLeftCounts = leftEncoder.Counts();
        Sleep(sleepTime);
    }

    rightMotor.Stop();
    leftMotor.Stop();

    std::string message = "Drove " + std::to_string(distance) + " in";
    LCD.WriteLine(message.c_str());
}