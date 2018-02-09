cmake_minimum_required(VERSION 2.8.9)
project(BlueCode)

# Compile against c++ 11
set(CMAKE_CXX_STANDARD 11)

set(SIM true CACHE BOOL "Compile with a plant model and a simulated vehicle")
set(DEBUG false CACHE BOOL "Compile without std::chrono dependencies (simple counter for clock)")
set(USE_CAMERA false CACHE BOOL "Use the default camera for actual photos. REQUIRES OPENCV.")

SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

# Compile the plant/vehicle model.
if(${SIM})
  message(STATUS "Compiling #SIM mode.")
  FIND_PACKAGE(OpenGL)
  FIND_PACKAGE(Irrlicht)

  IF (APPLE)
  	FIND_LIBRARY(CARBON_LIBRARY Carbon)
  	FIND_LIBRARY(COCOA_LIBRARY Cocoa)
  	FIND_LIBRARY(IOKIT_LIBRARY IOKit)
  	SET(OSX_LIBRARIES ${CARBON_LIBRARY} ${COCOA_LIBRARY} ${IOKIT_LIBRARY})
  ENDIF (APPLE)

  if(UNIX AND NOT APPLE)
    SET(LINUX_LIBRARIES X11 Xxf86vm )
  endif()

  if (MSVC_IDE)
      # hack to get around the "Debug" and "Release" directories cmake tries to add on Windows
      set_target_properties (${NAME} PROPERTIES PREFIX "../")
      SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG "${OUTPUT_DIRECTORY}")
      SET( CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "${OUTPUT_DIRECTORY}")
      FILE(COPY ${PROJECT_SOURCE_DIR}/irrlicht/bin/Win64-VisualStudio/Irrlicht.dll DESTINATION ${PROJECT_SOURCE_DIR}/build)
  endif()

  add_definitions(-D SIM)
  INCLUDE_DIRECTORIES(${IRRLICHT_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR})
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

add_executable(BlueCode WIN32 ${SOURCES})
target_link_libraries( BlueCode ${IRRLICHT_LIBRARY} ${OpenCV_LIBS} ${OSX_LIBRARIES} ${LINUX_LIBRARIES} ${OPENGL_LIBRARIES} )
