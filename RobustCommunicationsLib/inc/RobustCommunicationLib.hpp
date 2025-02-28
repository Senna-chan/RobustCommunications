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
            for (auto definition : definitions)
            {
                // if (definition.dataStringLayout != NULL)
                // {
                //     //free(definition.sscanfFormat);
                // }
            }
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
    bool singleThreadLoop();


    /**
     * Used to attach the functions to read/write the data.
     *
     * \param access Callback struct
     */
    void attachHardwareAccess(HardwareAccess access);

    void addCommandDefinition(CommandDefinition* definition);

    void printHelp();

private:
    /**
     * Logic to read a binary packet.
     *
     */
    void readForBinaryPacket();

    /**
     * Logic to read a char packet.
     *
     */
    void readForCharPacket();

    void parseBinaryPacket();

    void parseCharPacket();

    enum readState
    {
        READING_HEADER,
        READING_MODULE,
        READING_COMMAND,
        READING_STATUS,
        READING_DATASIZE,
        READING_CRC,
        READING_DATA,
        READING_FOOTER,
        READING_DONE
    };

    void writeBinaryPacket(BinaryPacket* packet);
    void writeCharPacket(CharPacket* packet);

    readState currentReadState = READING_HEADER;
    inline static std::array<CommandDefinition,definitionSize> definitions;
    static uint16_t freeDefinitionIndex;
    static uint8_t instanceAmount;

    uint8_t transmittingPacketNumber = 0;
    uint8_t receivedPacketNumber = 0;
    HardwareAccess hardware;
    uint8_t dataReadBuffer[1040] = { 0 }; // Ugly but usefull
    uint16_t dataReadBufferIndex = 0;
};
};
#endif