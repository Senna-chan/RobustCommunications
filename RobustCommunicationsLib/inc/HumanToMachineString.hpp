#ifndef _ROBUSTCOMMUNICATIONSHUMANTOMACHINESTRING_H
#define _ROBUSTCOMMUNICATIONSHUMANTOMACHINESTRING_H

#include <stdint.h>

#include "virt_array.hpp"

namespace RobustCommunications{
struct strTypeToFormat{
    char shortName[5];
    char format[8];
    uint8_t byteSize;
};

using HumanToMachinePtr = va::make_virt_array<strTypeToFormat, 
    {"u8", "%hhu", 1},
    {"s8", "%hhd", 1},
    {"u16", "%hu", 2},
    {"s16", "%hd", 2},
    {"u32", "%u", 4},
    {"s32", "%d", 4},
    {"u64", "%lu", 8},
    {"s64", "%ld", 8},
    {"int", "%d", 4},
    {"f", "%f", 8},
    {"d", "%f", 8},
    {"hex", "0x%02X", 2},
    {"s", "%s", 0},
    {"c", "%c", 1}
>;

inline static int findIndexByHumanReadable(std::string str){
    const auto dat = HumanToMachinePtr(0).get_full_data();
    for(int i = 0; i < dat.size(); i++){
        if(str == std::string(dat[i].shortName))
        {
            return i;
        }
    }
    return -1;
}

};
#endif
