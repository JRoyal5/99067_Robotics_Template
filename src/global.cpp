#include "main.h"

map<string, int> motors = {{"conveyerbelt", 4}, {"intake", 5}};
vector<int8_t> ports = {4, 5};

Robot robot({10, 8, 6}, {9, 7, 5}, 18, 19, ports);
