#ifndef _SERIAL_COMMS_H
#define _SERIAL_COMMS_H

#ifdef PIO_UNIT_TESTING
#include <unity.h>
#elif defined __ARDUINO__
#include "Arduino.h"
#endif

#include <cstring>
#include <functional>
#include <vector>
#include <stdint.h>

#include "Config.hpp"
#include "BinaryPacket.hpp"
#include "CharPacket.hpp"
#include "CommandDefinition.hpp"

namespace RobustCommunications{
class RobustCommunication
{
protected: 

public:
    RobustCommunication()
    {
        instanceAmount++;
    }

    ~RobustCommunication()
    {
        instanceAmount--;
        if (instanceAmount == 0)
        {
            
        }
    }

    struct HardwareAccess
    {
        std::function<int()> available;
        std::function<int()> read;
        std::function<int()> peek;
        std::function<int(uint8_t)> write;
    };

    /**
     * @brief When not using a RTOS we use this to get data, process it and run the callback
     *
     * @returns True if packet is handled
     */
    bool read();

    /**
     * Used to attach the functions to read/write the data.
     *
     * \param access Callback struct
     */
    void attachHardwareAccess(HardwareAccess access);

    void addCommandDefinition(CommandDefinition* definition);

    void printHelp();

private:

    void parseBinaryPacket();
    void parseCharPacket();

    void writeBinaryPacket(BinaryPacket* packet);
    void writeCharPacket(CharPacket* packet);

    inline static std::vector<CommandDefinition> definitions;
    inline static uint16_t freeDefinitionIndex = 0;
    inline static uint8_t instanceAmount = 0;

    uint8_t transmittingPacketNumber = 0;
    uint8_t receivedPacketNumber = 0;
    HardwareAccess hardware;
    std::array<uint8_t, MaxPacketBytes> dataReadBuffer = { 0 };
    uint16_t dataReadBufferIndex = 0;
    
};
};
#endif