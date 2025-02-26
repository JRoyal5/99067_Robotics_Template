#include "main.h"

map<string, int> motors = {
    {"conveyerbelt", 4}, 
    {"intake", 5}
    };
vector<int8_t> ports = {4, 5};

Robot robot(
    //Left drive motors
    {-10, -8, -6}, 
    //Right drive motors
    {9, 7, 5}, 
    //inertial sensors
    18, 19, 
    //assorted ports
    {4, 5}
    );

void motorMove(string name, int velocity){
    robot.assorted[motors[name]].move_velocity(velocity);
}