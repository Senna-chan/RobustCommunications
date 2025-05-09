#include <gtest/gtest.h>
#include "RobustCommunicationLib.hpp"

using namespace RobustCommunications;

struct TestData
{
    BinaryPacket packet;
    CharPacket cpacket;
    uint8_t* packetBytes;
    uint16_t packetSize;
};


uint16_t channelValue = 0;
uint16_t channelNumber = 0;
const uint16_t TestChannelValue = 1500;

bool setChannel(BinaryPacket* packet)
{
    if (packet->dataSize != 3) return false;
    std::tie(channelNumber, channelValue) = packet->data.unpack<uint8_t, uint16_t>();

    if (channelNumber != 1) return false;
    if (channelValue != 1500) return false;

    return true;
}

bool getChannel(BinaryPacket* packet)
{
    if(packet->data[0] == 1)
    {
        packet->data.pack(TestChannelValue);
        packet->status.ack = 1;
        return true;
    }
    return false;
}


class CommonTestFixture : public testing::Test {

protected:
    static const int returnPacketMaxSize = 2048;
    RobustCommunication rc;
    uint8_t returnedPacket[returnPacketMaxSize];
    uint16_t writeIndex;

    uint16_t readIndex;

    TestData testData;
    int rc_read()
    {
        return testData.packetBytes[readIndex++];
    }
    int rc_write(uint8_t b)
    {
        returnedPacket[writeIndex++] = b;
        return 1;
    }
    int rc_peek()
    {
        if (rc_available())
        {
            return testData.packetBytes[readIndex];
        }
        else
        {
            return 0;
        }
    }
    int rc_available()
    {
        if (readIndex < testData.packetSize)
        {
            return testData.packetSize - readIndex;
        }
        return 0;
    }

    void processPacket(TestData dataToTest)
    {
        testData = dataToTest;
        while (rc_available())
        {
            rc.read();
        }
    }

    void attachHardware()
    {
        RobustCommunication::HardwareAccess access;
        access.read = std::bind(&CommonTestFixture::rc_read, this);
        access.peek = std::bind(&CommonTestFixture::rc_peek, this);
        access.write = std::bind(&CommonTestFixture::rc_write, this, std::placeholders::_1);
        access.available = std::bind(&CommonTestFixture::rc_available, this);
        rc.attachHardwareAccess(access);
    }

    void attachTestDefinitions() 
    {
        CommandDefinition setChannelPacket;
        setChannelPacket.setBinaryInfo(0x01,0x01);
        setChannelPacket.setStringInfo("CC","setchannel");
        setChannelPacket.setIncomingDataStringLayout("u8,u16");
        setChannelPacket.commandFunction = setChannel;
        rc.addCommandDefinition(&setChannelPacket);

        CommandDefinition getChannelPacket;
        getChannelPacket.setBinaryInfo(0x01,0x02);
        getChannelPacket.setStringInfo("CC","getchannel");
        getChannelPacket.setIncomingDataStringLayout("u8");
        getChannelPacket.setOutgoingDataStringLayout("u16");
        getChannelPacket.commandFunction = getChannel;
        rc.addCommandDefinition(&getChannelPacket);
    }

    void SetUp() override {
        rc = RobustCommunication();
        attachHardware();
        writeIndex = 0;
        readIndex = 0;
        memset(returnedPacket, 0, returnPacketMaxSize);
    }

    void TearDown() override {
        channelValue = 0;
        channelNumber = 0;
    }
};

class CharTestFixture : public CommonTestFixture {
protected:
    CharTestFixture() {
    }

    CharPacket createCharPacket(const char* moduleName, const char* command, const char* data, bool getter)
    {
        CharPacket charPacket;
        charPacket.header[0] = '@';
        charPacket.header[1] = getter ? '<' : '>';
        charPacket.footer = '\n';
        strcpy(charPacket.moduleName, moduleName);
        strcpy(charPacket.commandName, command);
        strcpy(charPacket.data, data);
        return charPacket;
    }
};

class BinaryTestFixture : public CommonTestFixture {
protected:
    BinaryTestFixture() {

    }
};

TEST_F(BinaryTestFixture, test_binary_packet_to_array)
{
    TestData testData;
    testData.packet.header = 0xCC44;
    testData.packet.moduleClass = 0x01;
    testData.packet.command = 0x01;
    testData.packet.dataSize = 1;
    testData.packet.data.pack(uint8_t{0x01});
    testData.packetSize = BinaryPacket::InformationSize + 1;
    testData.packetBytes = (uint8_t*)malloc(testData.packetSize);
    testData.packet.toArray(testData.packetBytes);
    ASSERT_EQ(0x01, testData.packetBytes[10]);
    ASSERT_EQ(0xCC, testData.packetBytes[0]);
    ASSERT_EQ(0x44, testData.packetBytes[1]);
}

TEST_F(CharTestFixture, test_char_packet_to_array)
{
    TestData testData;
    testData.cpacket = createCharPacket("CC", "setchannel", "1,1500", false);
    testData.packetSize = 40;
    testData.packetBytes = (uint8_t*)malloc(testData.packetSize);
    testData.cpacket.toArray(testData.packetBytes);
    testData.packetSize = strlen((char*)testData.packetBytes);
    ASSERT_EQ(testData.packetSize, 24) << (char*)testData.packetBytes;
    ASSERT_STREQ((char*)testData.packetBytes, "@>CC|setchannel|1,1500|\n");
}

TEST_F(BinaryTestFixture, set_via_bin_packet)
{
    TestData testData;
    testData.packet.header = 0xCC44;
    testData.packet.moduleClass = 0x01;
    testData.packet.command = 0x01;
    testData.packet.dataSize = 3;
    testData.packet.data.pack(uint8_t{0x01}, uint16_t{TestChannelValue});
    testData.packetSize = BinaryPacket::InformationSize + 3;
    testData.packetBytes = (uint8_t*)malloc(testData.packetSize);
    testData.packet.toArray(testData.packetBytes);
    attachTestDefinitions();

    processPacket(testData);
    ASSERT_EQ(channelValue, 1500);
}

TEST_F(BinaryTestFixture, get_via_bin_packet)
{

    TestData testData;
    testData.packet.header = 0xCC44;
    testData.packet.moduleClass = 0x01;
    testData.packet.command = 0x02;
    testData.packet.dataSize = 1;
    testData.packet.data.pack(uint8_t{0x01});
    testData.packet.status.requestType = 1;
    testData.packetSize = BinaryPacket::InformationSize + 1;
    testData.packetBytes = (uint8_t*)malloc(testData.packetSize);
    testData.packet.toArray(testData.packetBytes);
    attachTestDefinitions();

    processPacket(testData);
}

TEST_F(CharTestFixture, set_via_char_packet)
{
    TestData testData;
    testData.cpacket = createCharPacket("CC", "setchannel", "1,1500", false);
    testData.packetSize = 40;
    testData.packetBytes = (uint8_t*)malloc(testData.packetSize);
    testData.cpacket.toArray(testData.packetBytes);
    testData.packetSize = strlen((char*)testData.packetBytes);

    attachTestDefinitions();

    processPacket(testData);

    //ASSERT_FALSE(testData.packet.status.internalError);
    //ASSERT_TRUE(testData.packet.status.ack);
    ASSERT_EQ(channelValue, 1500);
}

TEST_F(CharTestFixture, get_via_char_packet)
{
    TestData testData;
    testData.cpacket = createCharPacket("CC", "getchannel", "1", true);
    testData.packetSize = 40;
    testData.packetBytes = (uint8_t*)malloc(testData.packetSize);
    testData.cpacket.toArray(testData.packetBytes);
    testData.packetSize = strlen((char*)testData.packetBytes);

    attachTestDefinitions();
    processPacket(testData);
    //ASSERT_FALSE(testData.packet.status.internalError);
    //ASSERT_TRUE(testData.packet.status.ack);
    ASSERT_STREQ((char*)returnedPacket, "@=CC|getchannel|1500|\n");
}