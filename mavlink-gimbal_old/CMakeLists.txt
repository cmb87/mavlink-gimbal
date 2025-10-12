cmake_minimum_required(VERSION 3.10)
project(mavlink_gimbal)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add executable for the gimbal program
add_executable(gimbal main.cpp)

# Link MAVSDK library
target_link_libraries(gimbal)

# Optional: Include directories if needed
# target_include_directories(gimbal PRIVATE ${MAVSDK_INCLUDE_DIRS})