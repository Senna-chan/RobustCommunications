#ifndef _ROBUSTCOMMUNICATIONSCHARPACKET_H
#define _ROBUSTCOMMUNICATIONSCHARPACKET_H

#include <string>
#include <array>
#include <functional>
#include <vector>
#include <stdint.h>

#include "Config.hpp"

namespace RobustCommunications{
class CharPacket{
 public:
    CharPacket(){}
    char header[2] = {0};
    char moduleName[10] = {0};
    char commandName[30] = {0};
    char data[MaxDataBytes] = {0};
    char footer = 0;
    CharPacket arrayToPacket();
    void toArray(uint8_t* buffer, bool ignoreData = false);
    std::string toString(bool ignoreData = false);
    bool fromString(std::string str);

};
}
#endif