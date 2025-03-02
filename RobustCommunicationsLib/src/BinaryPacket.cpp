#include <cstring>

#include "BinaryPacket.hpp"

using namespace RobustCommunications;

void BinaryPacket::toArray(uint8_t* buffer, bool ignoreData){
    uint16_t bufferIndex = 0;
	buffer[bufferIndex++] = HeaderBytes >> 8;
	buffer[bufferIndex++] = HeaderBytes & 0xFF;
	buffer[bufferIndex++] = moduleClass;
	buffer[bufferIndex++] = command;
	buffer[bufferIndex++] = status.binary >> 8;
	buffer[bufferIndex++] = status.binary & 0xFF;
	buffer[bufferIndex++] = (dataSize >> 8);
	buffer[bufferIndex++] = (dataSize & 0xFF);
	buffer[bufferIndex++] = (crc >> 8);
	buffer[bufferIndex++] = (crc & 0xFF);
	DEBUGPRINTF("info: ");
	for(int i = 0; i < bufferIndex; i++)
	{
		DEBUGPRINTF("0x%02X ", buffer[i]);
	}
	if (!ignoreData)
	{
		std::memcpy(&buffer[bufferIndex], data.data.begin(), dataSize);
	}
	DEBUGPRINTF("\tbdata: ");
	for(int i = bufferIndex; i < bufferIndex + dataSize; i++)
	{
		DEBUGPRINTF("0x%02X ", buffer[i]);
	}
	DEBUGPRINTF("\tpdata: ");
	for(int i = 0; i < dataSize; i++)
	{
		DEBUGPRINTF("0x%02X ", data[i]);
	}
	DEBUGPRINTF("\n");
}

BinaryPacket BinaryPacket::arrayToPacket(std::span<uint8_t> packetBuffer, bool ignoreData){
	// Unpack here with Fiona's library later. Now we do it the harder way
	size_t bufferIndex = 0;
	DataPackerUnpacker dpu;
	header = packetBuffer[bufferIndex++] << 8;
	header += packetBuffer[bufferIndex++];
	moduleClass = packetBuffer[bufferIndex++];
	command = packetBuffer[bufferIndex++];
	status.binary = packetBuffer[bufferIndex++] << 8;
	status.binary += packetBuffer[bufferIndex++];
	dataSize = packetBuffer[bufferIndex++] << 8;
	dataSize += packetBuffer[bufferIndex++];
	crc = packetBuffer[bufferIndex++] << 8;
	crc += packetBuffer[bufferIndex++];
	auto subSpan = packetBuffer.subspan(bufferIndex, packetBuffer.size() - bufferIndex);
	std::memcpy(data.data.begin(), subSpan.data(), subSpan.size());
	return *this;
}