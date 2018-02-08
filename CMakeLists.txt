cmake_minimum_required(VERSION 2.8.9)
project(BlueCode)

# Compile against c++ 11
set(CMAKE_CXX_STANDARD 11)

# Find OpenCV
find_package( OpenCV REQUIRED )

# Bring in all of my headers
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(include)

# Add all sources to the executable
file(GLOB SOURCES "src/*.cpp")

add_executable(BlueCode ${SOURCES})
target_link_libraries( BlueCode ${OpenCV_LIBS} )
