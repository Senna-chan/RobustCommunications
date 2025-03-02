#include "RobustCommunicationLib.hpp"

using namespace RobustCommunications;

#define IGNORE_CRC

const uint8_t maxReadCyclesPerCall = 20;
uint8_t readCycleCounter = maxReadCyclesPerCall;
bool packetIsBinary = true;
BinaryPacket currentBinaryPacket;

CharPacket currentCharPacket;
uint16_t charPacketIndex = 0; /*<! Used to keep track of seperate buffers of the char packet */

uint16_t dataBytesToRead = 0;

typedef union {
    uint8_t u8[8];
    char c[8];
    uint16_t u16[4];
    uint16_t u32[2];
    uint16_t u64;
    float f;
    double d;
} byte8union;

const char* readStateStr[] = {
    "READING_HEADER",
    "READING_MODULE",
    "READING_COMMAND",
    "READING_STATUS",
    "READING_DATASIZE",
    "READING_CRC",
    "READING_DATA",
    "READING_FOOTER",
    "READING_DONE",
};
/**
 * Only used when we read binary data. We know how much we need to read.
 * With char buffer we just read until we see '|\n'
 */
static uint16_t dataReadAmount = 0;

bool RobustCommunication::read() {
    bool ret = false;
    while (hardware.available()) {
        if (readCycleCounter == 0) {
            readCycleCounter = maxReadCyclesPerCall;
            break;
        }
        readCycleCounter--;
        int b = hardware.read();
        int p = hardware.peek();
        if (dataReadBufferIndex == BinaryPacket::HEADERINDEX) {
            dataReadBuffer.fill(0);
            if (b == 0xCC && p == 0x44) {
                packetIsBinary = true;
            } else if (b == '@') {
                packetIsBinary = false;
            } else {
                throw std::runtime_error("First byte is not expected");
            }
        }

        dataReadBuffer[dataReadBufferIndex] = b;
        if (packetIsBinary) {
			if(dataReadAmount > 0 && dataReadBufferIndex >= BinaryPacket::DATAINDEX)
			{
				dataReadAmount--;
				if(dataReadAmount == 0)
				{
                    dataReadBufferIndex += 1;
					parseBinaryPacket();
                    ret = true;
				}
			}
            if (dataReadBufferIndex == BinaryPacket::DATASIZEINDEX) {
                dataReadAmount = b << 8 | p;
				b = hardware.read();
				dataReadBuffer[++dataReadBufferIndex] = b;
            }
        }
		else {
			if(b == '|' && p == '\n')
			{
				b = hardware.read();
				dataReadBuffer[++dataReadBufferIndex] = b;
				parseCharPacket();
                ret = true;
			}
		}
        dataReadBufferIndex++;
    }
    if(ret)
    {
        dataReadBufferIndex = 0;
        currentBinaryPacket = BinaryPacket();
        dataReadBuffer.fill(0);
        dataReadAmount = 0;
    }
    return ret;
}

void RobustCommunication::parseBinaryPacket() {
	currentBinaryPacket.arrayToPacket(std::span{dataReadBuffer}.subspan(0,dataReadBufferIndex));
    bool moduleFound = false;
    bool commandFound = false;
    int defIndex;
    for (defIndex = 0; defIndex < freeDefinitionIndex; defIndex++) {
        if (definitions[defIndex].moduleClass == currentBinaryPacket.moduleClass) {
            moduleFound = true;
            if (definitions[defIndex].commandClass == currentBinaryPacket.command) {
                commandFound = true;
            }
        }
        if (moduleFound && commandFound) {
            break;
        }
    }

    if (moduleFound && commandFound) {
        if (!definitions[defIndex].commandFunction(&currentBinaryPacket)) {
            currentBinaryPacket.status.internalError = 1;
        }
    }

    if (!moduleFound) {
        currentBinaryPacket.status.unknownModule = 1;
    } else if (!commandFound) {
        currentBinaryPacket.status.unknownCommand = 1;
    }

    if (!currentBinaryPacket.status.oneshot) {
        if (!currentBinaryPacket.status.requestType) {
            currentBinaryPacket.dataSize = 0;
        }
    }

    writeBinaryPacket(&currentBinaryPacket);
}

void RobustCommunication::parseCharPacket() {
    currentCharPacket.arrayToPacket(std::span{dataReadBuffer}.subspan(0,dataReadBufferIndex));
    bool moduleFound = false;
    bool commandFound = false;
    int defIndex;
    CommandDefinition* foundDefinition;
    for (defIndex = 0; defIndex < freeDefinitionIndex; defIndex++) {
        if (definitions[defIndex].moduleName == std::string(currentCharPacket.moduleName)) {
            moduleFound = true;
            if (definitions[defIndex].commandName == std::string(currentCharPacket.commandName)) {
                commandFound = true;
            }
        }
        if (moduleFound && commandFound) {
            foundDefinition = &definitions[defIndex];
            currentBinaryPacket.command = foundDefinition->commandClass;
            currentBinaryPacket.moduleClass = foundDefinition->commandClass;
            break;
        }
    }

    if (moduleFound && commandFound) {
        currentBinaryPacket.dataSize = 0;
        currentBinaryPacket.data.clear();
        uint8_t* bufPtr = currentBinaryPacket.data.data.begin();
        const char* data = currentCharPacket.data;
        const char* dataP = data;
        for (auto formatToken : foundDefinition->getIncomingFormat()) {
            char format[20] = {0};
            char* formatP = format;

            strcpy(formatP, formatToken->format);
            formatP += strlen(formatToken->format);

            strcpy(formatP, "%n%[^,]");

            int charsRead = 0;
            int sscanfResult = sscanf(dataP, format, bufPtr, &charsRead);
            if (sscanfResult != 1) {
                DEBUGPRINTF("Something went wrong\n");
                break;
            }
            if (charsRead == 0) {
                DEBUGPRINTF("Uh... didn't read anything\n");
            }
            dataP += charsRead;
            bufPtr += charsRead;
            if (format[1] == 's')  // We have read a string
            {
                currentBinaryPacket.dataSize += charsRead;
            } else {
                currentBinaryPacket.dataSize += formatToken->byteSize;
            }
            if (*dataP == ',') {
                dataP++;
            }
        }
        // currentBinaryPacket.dataSize += foundDefinition->expectedsscanfDataSize;
        currentBinaryPacket.status.ack = 1;

        if (!foundDefinition->commandFunction(&currentBinaryPacket)) {
            currentBinaryPacket.status.internalError = 1;
        } else {
            if (currentBinaryPacket.status.requestType == BinaryPacket::RequestType::GETTER) {
                currentBinaryPacket.dataSize = 0;
                uint8_t* bufPtr = currentBinaryPacket.data.data.begin();
                memset(currentCharPacket.data, 0, MaxDataBytes);
                char* dataP = currentCharPacket.data;

                for (auto formatToken : foundDefinition->getOutgoingFormat()) {
                    int sprintfResult = 0;
                    if (strcmp(formatToken->shortName, "s")) {
                        byte8union bu = {0};
                        memcpy(&bu, bufPtr, 8);
                        sprintfResult = sprintf(dataP, formatToken->format, bu);
                    } else {
                        sprintfResult = sprintf(dataP, formatToken->format, bufPtr);
                    }
                    if (sprintfResult == 0) {
                        DEBUGPRINTF("Failure writing data\n");
                    }
                    dataP += sprintfResult;
                    bufPtr += formatToken->byteSize;
                }
                currentCharPacket.header[0] = '@';
                currentCharPacket.header[1] = '=';
                currentCharPacket.footer = '\n';
                writeCharPacket(&currentCharPacket);
            }
        }
    }
}

void RobustCommunication::attachHardwareAccess(HardwareAccess access) {
    // if(access.read == nullptr)  Error_Handler();
    // if(access.write == nullptr)  Error_Handler();
    // if(access.available == nullptr)  Error_Handler();
    hardware = access;
}

void RobustCommunication::addCommandDefinition(CommandDefinition* definition) {
    if (freeDefinitionIndex != definitionSize) {
        definitions[freeDefinitionIndex++] = *definition;
    }
}

void RobustCommunication::printHelp() {
    DEBUGPRINTF("Available commands: \n");
    for (int i = 0; i < freeDefinitionIndex; i++) {
        if (!definitions[i].moduleName.empty()) {
            DEBUGPRINTF("- ASCII: module %s, command %s, dataformat %s ", definitions[i].moduleName, definitions[i].commandName, definitions[i].incomingHRDataStringLayout);
            if (!definitions[i].shortDesc.empty()) {
                DEBUGPRINTF("%s", definitions[i].shortDesc);
            }
            DEBUGPRINTF("\n");
        }
    }
}

void RobustCommunication::writeBinaryPacket(BinaryPacket* packet) {
    uint8_t returnPacket[BinaryPacket::InformationSize];
    packet->toArray(returnPacket, true);
    for (int i = 0; i < BinaryPacket::InformationSize; i++) {
        hardware.write(returnPacket[i]);
    }
    for (int i = 0; i < packet->dataSize; i++) {
        hardware.write(packet->data.data[i]);
    }
}

void RobustCommunication::writeCharPacket(CharPacket* packet) {
    uint8_t returnPacket[2024] = {0};
    packet->toArray(returnPacket);
    for (uint8_t b : returnPacket) {
        if (b == '\0') {
            break;
        }
        hardware.write(b);
    }
}

uint16_t RobustCommunication::freeDefinitionIndex = 0;
uint8_t RobustCommunication::instanceAmount = 0;