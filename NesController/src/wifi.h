#include <string>
#include <sstream>
#include <map>
#include "buttonState.h"

void wifiSetup();
void connectToSocket();
void wifiLoop(std::map<std::string, ButtonState> message);