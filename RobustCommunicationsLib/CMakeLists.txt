# Set the project name
project (RobustCommunicationLib)

# Add a library with the above sources
file(GLOB source_files 
  "src/BinaryPacket.cpp"
  "src/CharPacket.cpp"
  "src/CommandDefinition.cpp"
  "src/RobustCommunicationLib.cpp"
)
file(GLOB header_files 
  "inc/Config.hpp"
  "inc/BinaryPacket.hpp"
  "inc/CharPacket.hpp"
  "inc/CommandDefinition.hpp"
  "inc/HumanToMachineString.hpp"
  "inc/RobustCommunicationLib.hpp"
  "inc/virt_array.hpp"
)
add_library(RobustCommunicationLib 
  ${source_files}
)

target_include_directories( ${PROJECT_NAME}
    PUBLIC ${PROJECT_SOURCE_DIR}/inc
)