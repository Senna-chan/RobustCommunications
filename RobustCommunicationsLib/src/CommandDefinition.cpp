#include "CommandDefinition.hpp"

using namespace RobustCommunications;

void CommandDefinition::setBinaryInfo(uint8_t moduleClass, uint8_t commandClass){
    this->moduleClass = moduleClass;
    this->commandClass = commandClass;
}

void CommandDefinition::setStringInfo(const char* moduleName, const char* commandName){
    this->moduleName = moduleName;
    this->commandName = commandName;
}

void CommandDefinition::setDescription(const char* description)
{
    this->shortDesc = description;
}

void CommandDefinition::setIncomingDataStringLayout(const char* dataString){
    incomingHRDataStringLayout = dataString;
    convertStringToTokens(incomingHRDataStringLayout, &sscanfFormat);
}

void CommandDefinition::setOutgoingDataStringLayout(const char* dataString){
    outgoingHRDataStringLayout = dataString;
    convertStringToTokens(outgoingHRDataStringLayout, &sprintfFormat);
}

std::vector<HumanToMachinePtr> CommandDefinition::getIncomingFormat(){
    return sscanfFormat;
}

std::vector<HumanToMachinePtr> CommandDefinition::getOutgoingFormat(){
    return sprintfFormat;
}

void CommandDefinition::convertStringToTokens(std::string format, std::vector<HumanToMachinePtr>* tokens){
    std::string tmp = format;
    size_t pos = 0;
    std::string token;
    int formatIndex;
    while ((pos = tmp.find(',')) != std::string::npos) {
        token = tmp.substr(0, pos);
        formatIndex = findIndexByHumanReadable(token);
        tokens->push_back(formatIndex);
        tmp.erase(0, pos + 1);
    }
    formatIndex = findIndexByHumanReadable(tmp);
    tokens->push_back(formatIndex);
}