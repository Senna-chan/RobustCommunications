#ifndef _ROBUSTCOMMUNICATIONSBINARYPACKET_H
#define _ROBUSTCOMMUNICATIONSBINARYPACKET_H

#include <array>
#include <functional>
#include <vector>
#include <stdint.h>

#include "Config.hpp"

namespace RobustCommunications{
class BinaryPacket{
    public:
        BinaryPacket(){}
        /**
         * Size of all the information bytes used.
         */
        static const uint16_t InformationSize = 10;
        /**
         * Header bytes
         */
        static const uint16_t HeaderBytes = 0xCC44;

        enum RequestType{
            GETTER = 0,
            SETTER = 1,
            RESPONSE = 2
        };
        uint16_t header;
        uint8_t moduleClass;
        uint8_t command;
        uint16_t dataSize;
        uint16_t crc;
        std::array<uint8_t, MaxDataBytes> data;

        BinaryPacket arrayToPacket(uint8_t* buffer, bool ignoreData = false);
        void toArray(uint8_t* buffer, bool ignoreData = false);
        RequestType getRequestType();
        union
        {
            uint16_t binary;
            struct{
                uint16_t requestType : 2;    // If 0 then it is a request. If 1 then it is a response. If being used with a char packet then 0 is setter, 1 is getter
                uint16_t ack : 1;    
                uint16_t internalError : 1;  // Error in the handling of data
                uint16_t unknownModule : 1; 
                uint16_t unknownCommand : 1;
                uint16_t crcFault : 1;
                uint16_t dataSizeFault : 1;
                uint16_t oneshot : 1;    // If set then handle and forget
            };
        } status;
    private:
};
};


#endif