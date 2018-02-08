cmake_minimum_required(VERSION 2.8.9)
project(BlueCode)

# Compile against c++ 11
set(CMAKE_CXX_STANDARD 11)

# Compile the plant/vehicle model.
if(${SIM})
  message(STATUS "Compiling #SIM mode.")
  add_definitions(-D SIM)
endif()

# Compile debug mode (no dependencies on std::chrono)
if(${DEBUG})
  message(STATUS "Compiling #DEBUG mode.")
  add_definitions(-D DEBUG)
  set(CMAKE_BUILD_TYPE Debug)
endif()

# Attempt to use an attached camera
if(${USE_CAMERA})
  message(STATUS "Camera usage requested. Checking for OpenCV...")
  find_package( OpenCV )
  if(OpenCV_FOUND)
    add_definitions(-D USE_CAMERA)
  else()
    message(WARNING "OpenCV not found. Disabling camera usage.")
  endif(OpenCV_FOUND)
endif()

# Bring in all of my headers
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(include)

# Add all sources to the executable
file(GLOB SOURCES "src/*.cpp")

add_executable(BlueCode ${SOURCES})
target_link_libraries( BlueCode ${OpenCV_LIBS} )
