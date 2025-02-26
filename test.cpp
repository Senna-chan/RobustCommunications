#include <gtest/gtest.h>
#include "RobustCommunicationLib.h"

struct TestData
{
    RobustCommunication::BinaryPacket packet;
    RobustCommunication::CharPacket cpacket;
    uint8_t* packetBytes;
    uint16_t packetSize;
};


uint16_t channelValue = 0;
uint16_t channelNumber = 0;
const uint16_t TestChannelValue = 1500;

bool setChannel(RobustCommunication::BinaryPacket* packet)
{
    if (packet->dataSize != 3) return false;

    channelNumber = packet->data[0];
    channelValue = packet->data[1] | packet->data[2] << 8;

    if (channelNumber != 1) return false;
    if (channelValue != 1500) return false;

    return true;
}

bool getChannel(RobustCommunication::BinaryPacket* packet)
{
    if(packet->data[0] == 1)
    {
        packet->data[1] = TestChannelValue >> 8;
        packet->data[0] = TestChannelValue & 0xFF;
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
            rc.singleThreadLoop();
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
        RobustCommunication::CommsDefinition setChannelPacket = { 0 };
        setChannelPacket.moduleClass = 0x01;
        setChannelPacket.moduleName = "CC";
        setChannelPacket.commandClass = 0x01;
        setChannelPacket.commandName = "setchannel";
        setChannelPacket.incomingHRDataStringLayout = "u8,u16";
        setChannelPacket.commandFunction = setChannel;
        rc.addCommsDefinition(&setChannelPacket);

        RobustCommunication::CommsDefinition getChannelPacket = { 0 };
        getChannelPacket.moduleClass = 0x01;
        getChannelPacket.moduleName = "CC";
        getChannelPacket.commandClass = 0x02;
        getChannelPacket.commandName = "getchannel";
        getChannelPacket.incomingHRDataStringLayout = "u8";
        getChannelPacket.outgoingHRDataStringLayout = "u16";
        getChannelPacket.commandFunction = getChannel;
        rc.addCommsDefinition(&getChannelPacket);
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
        testData = { 0 };
    }
};

class CharTestFixture : public CommonTestFixture {
protected:
    CharTestFixture() {
    }

    RobustCommunication::CharPacket createCharPacket(const char* moduleName, const char* command, const char* data, bool getter)
    {
        RobustCommunication::CharPacket charPacket;
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
    testData.packet.data = new uint8_t{ 0x01 };
    testData.packetSize = RobustCommunication::BinaryPacketInformationSize + 1;
    testData.packetBytes = (uint8_t*)malloc(testData.packetSize);
    RobustCommunication::binaryPacketToDataArray(&testData.packet, testData.packetBytes);
    ASSERT_EQ(0x01, testData.packetBytes[9]);
    ASSERT_EQ(0xCC, testData.packetBytes[0]);
    ASSERT_EQ(0x44, testData.packetBytes[1]);
}

TEST_F(CharTestFixture, test_char_packet_to_array)
{
    TestData testData;
    testData.cpacket = createCharPacket("CC", "setchannel", "1,1500", false);
    testData.packetSize = 40;
    testData.packetBytes = (uint8_t*)malloc(testData.packetSize);
    RobustCommunication::charPacketToDataArray(&testData.cpacket, testData.packetBytes);
    testData.packetSize = strlen((char*)testData.packetBytes);
    ASSERT_EQ(testData.packetSize, 23) << (char*)testData.packetBytes;
    ASSERT_STREQ((char*)testData.packetBytes, "@>CC|setchannel|1,1500\n");
}

TEST_F(BinaryTestFixture, set_via_bin_packet)
{
    TestData testData;
    testData.packet.header = 0xCC44;
    testData.packet.moduleClass = 0x01;
    testData.packet.command = 0x01;
    testData.packet.dataSize = 3;
    testData.packet.data = new uint8_t[3]{ 0x01, (uint8_t)(TestChannelValue & 0xFF), (uint8_t)(TestChannelValue >> 8) };
    testData.packetSize = RobustCommunication::BinaryPacketInformationSize + 3;
    testData.packetBytes = (uint8_t*)malloc(testData.packetSize);
    RobustCommunication::binaryPacketToDataArray(&testData.packet, testData.packetBytes);
    attachTestDefinitions();

    processPacket(testData);
    ASSERT_EQ(channelValue, 1500);
}

TEST_F(BinaryTestFixture, get_via_bin_packet)
{

    TestData testData;
    testData.packet.header = 0xCC44;
    testData.packet.moduleClass = 0x01;
    testData.packet.command = 0x01;
    testData.packet.dataSize = 1;
    testData.packet.data = new uint8_t[1]{ 0x01};
    testData.packet.status.requestType = 1;
    testData.packetSize = RobustCommunication::BinaryPacketInformationSize + 1;
    testData.packetBytes = (uint8_t*)malloc(testData.packetSize);
    RobustCommunication::binaryPacketToDataArray(&testData.packet, testData.packetBytes);
    attachTestDefinitions();

    processPacket(testData);
}

TEST_F(CharTestFixture, set_via_char_packet)
{
    TestData testData;
    testData.cpacket = createCharPacket("CC", "setchannel", "1,1500", false);
    testData.packetSize = 40;
    testData.packetBytes = (uint8_t*)malloc(testData.packetSize);
    RobustCommunication::charPacketToDataArray(&testData.cpacket, testData.packetBytes);
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
    RobustCommunication::charPacketToDataArray(&testData.cpacket, testData.packetBytes);
    testData.packetSize = strlen((char*)testData.packetBytes);

    attachTestDefinitions();
    processPacket(testData);
    //ASSERT_FALSE(testData.packet.status.internalError);
    //ASSERT_TRUE(testData.packet.status.ack);
    ASSERT_STREQ((char*)returnedPacket, "@=CC|getchannel|1500\n");
}