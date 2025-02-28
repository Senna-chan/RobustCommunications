#ifndef _ROBUSTCOMMUNICATIONSCOMMANDDEF_H
#define _ROBUSTCOMMUNICATIONSCOMMANDDEF_H

#include <functional>
#include <vector>
#include <stdint.h>

#include "HumanToMachineString.hpp"
#include "BinaryPacket.hpp"

namespace RobustCommunications{
class CommandDefinition{
    public:
        void setBinaryInfo(uint8_t moduleClass, uint8_t commandClass);
        void setStringInfo(const char* moduleName, const char* commandName);
        void setDescription(const char* description);

        uint8_t moduleClass;    //<! Byte based module class
        uint8_t commandClass;   //<! Byte based command class
        
        std::string moduleName; //<! String based module class
        std::string commandName; //<! String based command class
        
        std::string shortDesc; //<! Short desciption for string based help
        
        std::function<bool(BinaryPacket*)> commandFunction; //<! Function to excecute
        
        void setIncomingDataStringLayout(const char* dataString);
        void setOutgoingDataStringLayout(const char* dataString);
        std::vector<HumanToMachinePtr> getIncomingFormat();
        std::vector<HumanToMachinePtr> getOutgoingFormat();
    private:
        std::string incomingHRDataStringLayout; //<! Incoming human readable data layout
        std::string outgoingHRDataStringLayout; //<! Outgoing human readable data layout
        std::vector<HumanToMachinePtr> sscanfFormat; //<! Incoming machine readable data layout indexes
        std::vector<HumanToMachinePtr> sprintfFormat; //<! Outgoing machine readable data layout indexes
        uint16_t expectedsscanfDataSize; //<! Expected incoming datasize
        uint16_t expectedsprintfDataSize; //<! Expected outgoing datasize
        void convertStringToTokens(std::string format, std::vector<HumanToMachinePtr>* tokens);
};
};


#endif