#ifndef _ROBUSTCOMMUNICATIONSBINARYPACKET_H
#define _ROBUSTCOMMUNICATIONSBINARYPACKET_H

#include <array>
#include <functional>
#include <vector>
#include <span>
#include <stdint.h>

#include "Config.hpp"
#include "DataPackerUnpacker.hpp"

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
        // Package order! KEEP IN SYNC
        uint16_t header = 0;
        uint8_t moduleClass = 0;
        uint8_t command = 0;
        union
        {
            uint16_t binary = 0;
            struct{
                uint16_t requestType : 2;    // See RequestType enum for what is what
                uint16_t ack : 1;    
                uint16_t internalError : 1;  // Error in the handling of data
                uint16_t unknownModule : 1; 
                uint16_t unknownCommand : 1;
                uint16_t crcFault : 1;
                uint16_t dataSizeFault : 1;
                uint16_t oneshot : 1;    // If set then handle and forget
            };
        } status;
        uint16_t dataSize = 0;
        uint16_t crc = 0;
        DataPackerUnpacker data;
        //
        // Indexes
        static constexpr size_t HEADERINDEX = 0;
        static constexpr size_t MODULEINDEX = HEADERINDEX + sizeof(header);
        static constexpr size_t COMMANDINDEX = MODULEINDEX + sizeof(moduleClass);
        static constexpr size_t STATUSINDEX = COMMANDINDEX + sizeof(command);
        static constexpr size_t DATASIZEINDEX = STATUSINDEX + sizeof(status);
        static constexpr size_t CRCINDEX = DATASIZEINDEX + sizeof(dataSize);
        static constexpr size_t DATAINDEX = CRCINDEX + sizeof(crc);
        BinaryPacket arrayToPacket(std::span<uint8_t> packetBuffer, bool ignoreData = false);
        void toArray(uint8_t* buffer, bool ignoreData = false);
        RequestType getRequestType();
    private:
};
};


#endif