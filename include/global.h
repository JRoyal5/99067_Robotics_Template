#include "api.h"
#include "robot.h"
using namespace pros;
using namespace std;

#ifndef GLOBALS_H
#define GLOBALS_H

extern Robot robot;

extern double driveBaseRadius;
extern double gearRatio;
extern double wheelDiameter;
extern double tickPerRev;

extern int motorColor;

void drive(int left, int right);

void driveForward(double inches, double power);

void driveBack(double inches, double power);

void turn(int target, int Mpower);

void auton();

#endif