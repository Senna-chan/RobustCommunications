#include <cstring>

#include "CharPacket.hpp"

using namespace RobustCommunications;

void CharPacket::toArray(uint8_t* buffer, bool ignoreData)
{
	uint8_t* bufferPtr = buffer;
	*bufferPtr++ = header[0];
	*bufferPtr++ = header[1];

	memcpy(bufferPtr, moduleName, strlen(moduleName));
	bufferPtr += strlen(moduleName);

	*bufferPtr++ = charPacketSeperator;

	memcpy(bufferPtr, commandName, strlen(commandName));
	bufferPtr += strlen(commandName);
    if(!ignoreData){
        *bufferPtr++ = charPacketSeperator;

        memcpy(bufferPtr, data, strlen(data));
        bufferPtr += strlen(data);
    }
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

bool CharPacket::fromString(std::string str){
	bool ret = false;
}