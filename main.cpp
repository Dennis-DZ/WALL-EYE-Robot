#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHRPS.h>
#include <string>

DigitalEncoder rightEncoder(FEHIO::P0_0);
DigitalEncoder leftEncoder(FEHIO::P0_7);
const int countsPer6Inch = 242;
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

int main() {

    const int motorPercent = 25;
    const int fastPercent = 40;
    const double sleepTime = 0.5;

    float x, y;

    LCD.Clear(BLACK);
    LCD.SetFontColor(WHITE);

    LCD.WriteLine("Touch the screen");
    while (!LCD.Touch(&x,&y));
    while (LCD.Touch(&x,&y));

    while (cdsCell.Value() > 2) {
        LCD.WriteLine(cdsCell.Value());
    }

    turn(motorPercent, 4);
    Sleep(sleepTime);

    driveForward(motorPercent, 35);
    Sleep(sleepTime);

    turn(motorPercent, -94);
    Sleep(sleepTime);

    driveForward(motorPercent, 10);
    Sleep(sleepTime);

    turn(motorPercent, 90);
    Sleep(sleepTime);

    driveForward(motorPercent, 18);
    Sleep(sleepTime);

    backUp(motorPercent, 18);
    Sleep(sleepTime);

    turn(motorPercent, -90);
    Sleep(sleepTime);

    backUp(motorPercent, 10);
    Sleep(sleepTime);

    turn(motorPercent, 94);
    Sleep(sleepTime);

    backUp(motorPercent, 30);
    Sleep(sleepTime);

    return 0;
}
