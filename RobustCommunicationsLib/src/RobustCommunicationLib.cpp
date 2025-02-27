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

typedef union 
{
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



bool RobustCommunication::singleThreadLoop()
{
	while (hardware.available() || currentReadState == READING_DONE)
	{
		if (readCycleCounter == 0)
		{
			readCycleCounter = maxReadCyclesPerCall;
			return false;
		}
		readCycleCounter--;

		TRACEPRINTF("Current read state %s byte to read %c(0x%02X)\n", readStateStr[currentReadState], hardware.peek(), hardware.peek());

		switch (currentReadState)
		{
		case READING_HEADER:
		{
			int b = hardware.read();
			if (b == 0xCC)
			{
				int p = hardware.peek();
				if (p == 0x44)
				{
					p = hardware.read();
					currentBinaryPacket.header = b << 8 | p;
					currentReadState = READING_MODULE;
					packetIsBinary = true;
				}
				else
				{
					DEBUGPRINTF("Header not found correctly\n");
				}
			}
			else if (b == '@')
			{
				packetIsBinary = false;
				DEBUGPRINTF("Header found\n");
				int p = hardware.peek();
				if (p == '>')
				{
					DEBUGPRINTF("SETTER found");
					currentBinaryPacket.status.requestType = BinaryPacket::RequestType::SETTER;
				}
				else if (p == '<')
				{
					DEBUGPRINTF("GETTER found");
					currentBinaryPacket.status.requestType = BinaryPacket::RequestType::GETTER;
				}
				else if(p == '=')
				{
					DEBUGPRINTF("RESPONSE FOUND");
					currentBinaryPacket.status.requestType = BinaryPacket::RequestType::RESPONSE;
				} else{
					DEBUGPRINTF("ERROR: Char '%s' is not a request type\n", p);
				}
				hardware.read();
				currentReadState = READING_MODULE;
			}
			else if (b == '?')
			{
				if (hardware.peek() == '\n')
				{
					printHelp();
					currentReadState = READING_MODULE;
				}
			}
			else
			{
				DEBUGPRINTF("Header not found while in header state");
			}
		}
		break;

		default:
			if (packetIsBinary)
			{
				readForBinaryPacket();
			}
			else
			{
				readForCharPacket();
			}
		}
	}

	return true;
}

void RobustCommunication::readForBinaryPacket()
{
	switch (currentReadState)
	{
	case READING_HEADER:
	{
		DEBUGPRINTF("OI, this cannot happen");
	}
	break;

	case READING_MODULE:
	{
		currentBinaryPacket.moduleClass = dataReadBuffer[dataReadBufferIndex++] = hardware.read();
		currentReadState = READING_COMMAND;
	}
	break;

	case READING_COMMAND:
	{
		currentBinaryPacket.command = dataReadBuffer[dataReadBufferIndex++] = hardware.read();
		currentReadState = READING_STATUS;
	}
	break;

	case READING_STATUS:
	{
		int b = hardware.read();
		dataReadBuffer[dataReadBufferIndex++] = b;
		currentBinaryPacket.status.binary = b << 8;
		b = hardware.read();
		dataReadBuffer[dataReadBufferIndex++] = b;
		currentBinaryPacket.status.binary = b;
		currentReadState = READING_DATASIZE;
	}
	break;

	case READING_DATASIZE:
	{
		int b = hardware.read();
		currentBinaryPacket.dataSize = b << 8;
		dataReadBuffer[dataReadBufferIndex++] = b;
		b = hardware.read();
		currentBinaryPacket.dataSize |= b;
		dataReadBuffer[dataReadBufferIndex++] = b;
		dataBytesToRead = currentBinaryPacket.dataSize;
		currentReadState = READING_CRC;
		if (currentBinaryPacket.dataSize != 0)
		{
			currentBinaryPacket.data.clear();
		}
	}
	break;

	case READING_CRC:
	{
		uint8_t b = hardware.read();
		currentBinaryPacket.crc = b << 8;
		dataReadBuffer[dataReadBufferIndex++] = b;
		b = hardware.read();
		currentBinaryPacket.crc |= b;
		dataReadBuffer[dataReadBufferIndex++] = b;
		if (dataBytesToRead > 0)
		{
			currentReadState = READING_DATA;
		}
		else
		{
			currentReadState = READING_DONE;
		}
	}
	break;

	case READING_DATA:
	{
		for (int i = 0; i < maxReadCyclesPerCall; i++)
		{
			uint8_t b = hardware.read();
			uint8_t p = hardware.peek();
			dataReadBuffer[dataReadBufferIndex] = b;
			uint16_t dataBufferIndex = dataReadBufferIndex - (BinaryPacket::InformationSize - 2);
			currentBinaryPacket.data.pack(b);
			dataReadBufferIndex++;
			readCycleCounter--;
			dataBytesToRead--;
			if (dataBytesToRead == 0)
			{
				currentReadState = READING_DONE;
				break;
			}
		}

	}
	break;

	case READING_DONE:
	{
		parseBinaryPacket();
	}
	break;
	}
}

void RobustCommunication::readForCharPacket()
{
	switch (currentReadState)
	{
	case READING_HEADER:
	{
		DEBUGPRINTF("OI, this cannot happen");
	}
	break;

	case READING_MODULE:
	{
		currentCharPacket.moduleName[charPacketIndex++] = hardware.read();
		if (hardware.peek() == charPacketSeperator)
		{
			hardware.read();
			currentReadState = READING_COMMAND;
			charPacketIndex = 0;
		}
	}
	break;

	case READING_COMMAND:
	{
		currentCharPacket.commandName[charPacketIndex++] = hardware.read();
		if (hardware.peek() == charPacketSeperator)
		{
			hardware.read();
			currentReadState = READING_DATA;
			charPacketIndex = 0;
		}
	}
	break;

	case READING_STATUS:
	case READING_DATASIZE:
	case READING_CRC:
	{
		DEBUGPRINTF("OI, these states do not exist for char packets");
	}
	break;

	case READING_DATA:
	{
		currentCharPacket.data[charPacketIndex++] = hardware.read();
		if (hardware.peek() == '\n')
		{
			hardware.read();
			currentReadState = READING_DONE;
			charPacketIndex = 0;
		}
	}
	break;

	case READING_DONE:
	{
		parseCharPacket();
	}
	break;
	}
}

void RobustCommunication::parseBinaryPacket()
{
	bool moduleFound = false;
	bool commandFound = false;
	int defIndex;
	for (defIndex = 0; defIndex < freeDefinitionIndex; defIndex++)
	{
		if (definitions[defIndex].moduleClass == currentBinaryPacket.moduleClass)
		{
			moduleFound = true;
			if (definitions[defIndex].commandClass == currentBinaryPacket.command)
			{
				commandFound = true;
			}
		}
		if (moduleFound && commandFound)
		{
			break;
		}
	}

	if (moduleFound && commandFound)
	{
		if (!definitions[defIndex].commandFunction(&currentBinaryPacket))
		{
			currentBinaryPacket.status.internalError = 1;
		}
	}

	if (!moduleFound)
	{
		currentBinaryPacket.status.unknownModule = 1;
	}
	else if (!commandFound)
	{
		currentBinaryPacket.status.unknownCommand = 1;
	}

	if (!currentBinaryPacket.status.oneshot)
	{
		if (!currentBinaryPacket.status.requestType)
		{
			currentBinaryPacket.dataSize = 0;
		}
	}

	writeBinaryPacket(&currentBinaryPacket);
	currentReadState = READING_HEADER;
}

void RobustCommunication::parseCharPacket()
{
	bool moduleFound = false;
	bool commandFound = false;
	int defIndex;
	CommandDefinition *foundDefinition;
	for (defIndex = 0; defIndex < freeDefinitionIndex; defIndex++)
	{
		if (definitions[defIndex].moduleName == std::string(currentCharPacket.moduleName))
		{

			moduleFound = true;
			if (definitions[defIndex].commandName == std::string(currentCharPacket.commandName))
			{
				commandFound = true;
			}
		}
		if (moduleFound && commandFound)
		{
			foundDefinition = &definitions[defIndex];
			currentBinaryPacket.command = foundDefinition->commandClass;
			currentBinaryPacket.moduleClass = foundDefinition->commandClass;
			break;
		}
	}

	if (moduleFound && commandFound)
	{
		currentBinaryPacket.dataSize = 0;
		currentBinaryPacket.data.clear();
		uint8_t* bufPtr = currentBinaryPacket.data.data.begin();
		const char* data = currentCharPacket.data;
		const char* dataP = data;
		for (auto formatToken : foundDefinition->getIncomingFormat())
		{
			char format[20] = {0};
			char* formatP = format;

			strcpy(formatP, formatToken->format);
			formatP += strlen(formatToken->format);

			strcpy(formatP, "%n%[^,]");

			int charsRead = 0;
			int sscanfResult = sscanf(dataP, format, bufPtr, &charsRead);
			if (sscanfResult != 1)
			{
				DEBUGPRINTF("Something went wrong\n");
				break;
			}
			if (charsRead == 0)
			{
				DEBUGPRINTF("Uh... didn't read anything\n");
			}
			dataP += charsRead;
			bufPtr += charsRead;
			if (format[1] == 's') // We have read a string
			{
				currentBinaryPacket.dataSize += charsRead;
			} else {
				currentBinaryPacket.dataSize += formatToken->byteSize;
			}
			if (*dataP == ',')
			{
				dataP++;
			}
		}
		// currentBinaryPacket.dataSize += foundDefinition->expectedsscanfDataSize;
		currentBinaryPacket.status.ack = 1;

		if (!foundDefinition->commandFunction(&currentBinaryPacket))
		{
			currentBinaryPacket.status.internalError = 1;
		}
		else
		{
			if (currentBinaryPacket.status.requestType == BinaryPacket::RequestType::GETTER)
			{
				currentBinaryPacket.dataSize = 0;
				uint8_t* bufPtr = currentBinaryPacket.data.data.begin();
				memset(currentCharPacket.data, 0, MaxDataBytes);
				char* dataP = currentCharPacket.data;
				
				for (auto formatToken : foundDefinition->getOutgoingFormat())
				{
					int sprintfResult = 0;
					if(strcmp(formatToken->shortName, "s"))
					{
						byte8union bu = {0};
						memcpy(&bu, bufPtr, 8);
						sprintfResult = sprintf(dataP, formatToken->format, bu);
					}
					else
					{
						sprintfResult = sprintf(dataP, formatToken->format, bufPtr);
					}
					if (sprintfResult == 0)
					{
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
	currentReadState = READING_HEADER;
}

void RobustCommunication::attachHardwareAccess(HardwareAccess access)
{
	//if(access.read == nullptr)  Error_Handler();
	//if(access.write == nullptr)  Error_Handler();
	//if(access.available == nullptr)  Error_Handler();
	hardware = access;
}

void RobustCommunication::addCommandDefinition(CommandDefinition* definition)
{
	if (freeDefinitionIndex != definitionSize)
	{
		definitions[freeDefinitionIndex++] = *definition;
	}
}

void RobustCommunication::printHelp()
{
	DEBUGPRINTF("Available commands: \n");
	for (int i = 0; i < freeDefinitionIndex; i++)
	{
		if (!definitions[i].moduleName.empty())
		{
			DEBUGPRINTF("- ASCII: module %s, command %s, dataformat %s ", definitions[i].moduleName, definitions[i].commandName, definitions[i].incomingHRDataStringLayout);
			if (!definitions[i].shortDesc.empty())
			{
				DEBUGPRINTF("%s", definitions[i].shortDesc);
			}
			DEBUGPRINTF("\n");
		}
	}

}

void RobustCommunication::writeBinaryPacket(BinaryPacket* packet)
{
	uint8_t returnPacket[BinaryPacket::InformationSize];
	packet->toArray(returnPacket, true);
	for (int i = 0; i < BinaryPacket::InformationSize; i++)
	{
		hardware.write(returnPacket[i]);
	}
	for (int i = 0; i < packet->dataSize; i++)
	{
		hardware.write(packet->data.data[i]);
	}
}

void RobustCommunication::writeCharPacket(CharPacket* packet)
{
	uint8_t returnPacket[2024] = {0};
	packet->toArray(returnPacket);
	for (uint8_t b : returnPacket)
	{
		if(b == '\0') break;
		hardware.write(b);
	}
}

uint16_t RobustCommunication::freeDefinitionIndex = 0;
uint8_t RobustCommunication::instanceAmount = 0;