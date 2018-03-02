cmake_minimum_required(VERSION 2.8.9)
project(BlueCode)

# Compile against c++ 11
set(CMAKE_CXX_STANDARD 11)

set(TEST_PWM false CACHE BOOL "Test PWM code.")
set(USE_LASER false CACHE BOOL "Compile with laser logic included.")
set(SIM false CACHE BOOL "Compile with a plant model and a simulated vehicle")
set(USE_IRRLICHT false CACHE BOOL "Compile with a plant model and a simulated vehicle")
set(DEBUG false CACHE BOOL "Compile without std::chrono dependencies (simple counter for clock)")
set(USE_CAMERA false CACHE BOOL "Use the default camera for actual photos. REQUIRES OPENCV.")

SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

# Compile the plant/vehicle model.
if(${SIM})
  add_definitions(-D SIM)
  message(STATUS "Compiling #SIM mode.")

  # Compile the 3d engine along with the simulation.
  if(${USE_IRRLICHT})

    add_definitions(-D USE_IRRLICHT)
    message(STATUS "Adding #USE_IRRLICHT dependencies.")

    FIND_PACKAGE(OpenGL)
    FIND_PACKAGE(Irrlicht)
    message(STATUS "${IRRLICHT_LIBRARIES}")

    IF (APPLE)
      message(STATUS "Adding additional Apple libraries.")
    	FIND_LIBRARY(CARBON_LIBRARY Carbon)
    	FIND_LIBRARY(COCOA_LIBRARY Cocoa)
    	FIND_LIBRARY(IOKIT_LIBRARY IOKit)
    	SET(OSX_LIBRARIES ${CARBON_LIBRARY} ${COCOA_LIBRARY} ${IOKIT_LIBRARY})
    ENDIF (APPLE)

    if(UNIX AND NOT APPLE)
      message(STATUS "Adding additional linux libraries.")
      SET(LINUX_LIBRARIES X11 Xxf86vm )
    endif(UNIX AND NOT APPLE)

    if (MSVC_IDE)
        # hack to get around the "Debug" and "Release" directories cmake tries to add on Windows
        FILE(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/Debug)
        FILE(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/Release)
        message(STATUS "Running on windows. Copying over Irrlicht DLL...")
        FILE(COPY ${PROJECT_SOURCE_DIR}/irrlicht/bin/Win32-VisualStudio/Irrlicht.dll DESTINATION ${PROJECT_BINARY_DIR}/Debug)
        FILE(COPY ${PROJECT_SOURCE_DIR}/irrlicht/bin/Win32-VisualStudio/Irrlicht.dll DESTINATION ${PROJECT_BINARY_DIR}/Release)
    endif(MSVC_IDE)

    INCLUDE_DIRECTORIES(${IRRLICHT_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR})

  endif(${USE_IRRLICHT})

endif(${SIM})

# Compile laser logic.
if(${USE_LASER})
  message(STATUS "Compiling #USE_LASER code.")
  add_definitions(-D USE_LASER)
endif()

# Compile debug mode (no dependencies on std::chrono)
if(${DEBUG})
  message(STATUS "Compiling #DEBUG mode.")
  add_definitions(-D DEBUG)
  set(CMAKE_BUILD_TYPE Debug)
endif()

# Compile against threading.
if(${TEST_PWM})
  message(STATUS "Compiling #TEST_PWM packages.")
  add_definitions(-D TEST_PWM)
  set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
  set(THREADS_PREFER_PTHREAD_FLAG TRUE)
  find_package( Threads REQUIRED )
  set(CMAKE_C_FLAGS ${CMAKE_C_FLAGS} "-lpthread -lprussdrv")
endif()

# Attempt to use an attached camera
if(${USE_CAMERA})
  message(STATUS "Camera usage requested. Checking for V4L2...")
  FIND_PACKAGE(V4L2 REQUIRED)
  add_definitions(-D USE_CAMERA)
endif()

# Bring in all of my headers
include_directories(include)
include_directories( ${V4L2_INCLUDE_DIRS} )

# Add all sources to the executable
file(GLOB SOURCES "src/*.cpp")
set(source_files ${SOURCES})
if(NOT ${SIM})
  message(STATUS "Compiling for use on BeagleBone (no SIM)...")
  list(REMOVE_ITEM source_files "${PROJECT_SOURCE_DIR}/src/Vehicle.cpp")
  list(REMOVE_ITEM source_files "${PROJECT_SOURCE_DIR}/src/PlantModel.cpp")
endif()

add_executable(BlueCode WIN32 ${source_files})
target_link_libraries( BlueCode ${IRRLICHT_LIBRARY}
	${CMAKE_THREAD_LIBS_INIT}
	${V4L2_LIBRARIES}
	${OSX_LIBRARIES} ${LINUX_LIBRARIES} ${OPENGL_LIBRARIES} )

if(${TEST_PWM})
  target_link_libraries( BlueCode -lprussdrv )
endif()
