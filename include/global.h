#include "api.h"
#include "robot.h"
using namespace pros;
using namespace std;

#ifndef GLOBALS_H
#define GLOBALS_H

extern Robot robot;

extern map<string, int> motors;
extern std::vector<int8_t> ports;

void calibratePorts();

void auton();

#endif