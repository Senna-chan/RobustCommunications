cmake_minimum_required(VERSION 3.14)
project(my_project)

# GoogleTest requires at least C++14
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CONFIGURATION_TYPES "Debug" CACHE STRING "" FORCE)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)


enable_testing()

file(GLOB source_files 
  "test.cpp" 
  "RobustCommunicationLib.cpp"
)
file(GLOB header_files 
  "RobustCommunicationLib.h"
)

add_executable(
  RobustCommsTest
  ${source_files}

  ${header_files}
)

target_link_libraries(
  RobustCommsTest
  GTest::gtest_main
)
target_compile_features(RobustCommsTest PUBLIC cxx_std_23)
include(GoogleTest)
gtest_discover_tests(RobustCommsTest)
