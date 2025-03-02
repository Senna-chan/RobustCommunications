#include <cstring>

#include "CharPacket.hpp"

using namespace RobustCommunications;

void CharPacket::toArray(uint8_t* buffer, bool ignoreData)
{
	uint8_t* bufferPtr = buffer;
	*bufferPtr++ = header[0];
	*bufferPtr++ = header[1];

	std::memcpy(bufferPtr, moduleName, std::strlen(moduleName));
	bufferPtr += strlen(moduleName);

	*bufferPtr++ = charPacketSeperator;

	std::memcpy(bufferPtr, commandName, std::strlen(commandName));
	bufferPtr += std::strlen(commandName);
    if(!ignoreData){
        *bufferPtr++ = charPacketSeperator;

        std::memcpy(bufferPtr, data, std::strlen(data));
        bufferPtr += std::strlen(data);
    }
	*bufferPtr++ = charPacketSeperator;
	*bufferPtr++ = footer;
	*bufferPtr++ = '\0';

	DEBUGPRINTF("Databuffer is %s", (char*)buffer);
}

std::string CharPacket::toString(bool ignoreData)
{
	auto ret = std::string();
	ret.reserve(2 + 10 + 30 + 1);
	ret += header;
	ret += moduleName;
	ret += charPacketSeperator;
	ret += commandName;
	if(!ignoreData)
	{
		ret += charPacketSeperator;
		ret += data;
	}
	ret += footer;
	return ret;
}


CharPacket CharPacket::arrayToPacket(std::span<uint8_t> packetBuffer, bool ignoreData){
	size_t bufferIndex = 0;
	header[0] = packetBuffer[bufferIndex++];
	header[1] = packetBuffer[bufferIndex++];
	for(int i = 0; i < 10; i++){
		if(packetBuffer[bufferIndex] == charPacketSeperator) {bufferIndex++; break;}
		moduleName[i] = packetBuffer[bufferIndex++];
	}

	for(int i = 0; i < 30; i++){
		if(packetBuffer[bufferIndex] == charPacketSeperator)  {bufferIndex++; break;}
		commandName[i] = packetBuffer[bufferIndex++];
	}

	for(int i = 0; i < MaxDataBytes; i++){
		if(packetBuffer[bufferIndex] == charPacketSeperator)  {bufferIndex++; break;}
		data[i] = packetBuffer[bufferIndex++];
	}
	return *this;
}

bool CharPacket::fromString(std::string str){
	bool ret = false;

	return ret;
}