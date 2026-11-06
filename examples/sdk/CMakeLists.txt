cmake_minimum_required(VERSION 3.16)
project(SharpaMujocoNorthSDKExamples LANGUAGES CXX)

# Build independently of the simulator, using an installed official SDK.
find_package(NorthSDK 1.2 CONFIG REQUIRED)
foreach(example read_state home)
    add_executable(sdk_${example} ${example}.cpp)
    target_compile_features(sdk_${example} PRIVATE cxx_std_17)
    target_compile_options(sdk_${example} PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(sdk_${example} PRIVATE NorthSDK::NorthSDK)
endforeach()
