#include "main.h"

std::vector<int8_t> ports;
map<string, int> motors = {{"conveyerbelt", 4}, {"intake", 5}};
calibratePorts(){
    for(auto port:motors){
        ports.push_back(port.second);
    }
}

Robot robot({10, 8, 6}, {9, 7, 5}, 18, 19);
