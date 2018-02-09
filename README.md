# BlueCode
Coding base for AERSP 440's Blue Team (a.k.a. the winning team)

# Prerequisites
First, download and install CMake.

To use camera testing, you'll need OpenCV:

    If you are running on windows, don't forget to set your OpenCV_DIR environment variable. Look up directions online.

    For MACOSX and Linux and WSL, run all standard installation requirements for OpenCV.

To use Simulation testing:

    For MAC users, make sure you have the XCode development suite (just the tool library is enough.)

    For Windows users, nothing extra is required.

    For Linux/WSL users, run the following command:

        sudo apt-get -y install build-essential xserver-xorg-dev x11proto-xf86vidmode-dev libxxf86vm-dev mesa-common-dev libgl1-mesa-dev libglu1-mesa-dev libxext-dev libxcursor-dev

# CMake and Build
An option is to run cmake-gui. You can also control from command line. Do the following commands to start:

    Make a directory called "build" in the main BlueCode directory

    cd into "build"

    Run any of the following CMAKE commands

CMake and compile in Sim mode (needs Sim dependencies):

    cmake ..

CMake and compile in Sim mode, with camera (needs Sim dependencies & OpenCV):

    cmake -D USE_CAMERA=true ..

CMake and compile for use on BBB with no camera:

    cmake -D SIM=false ..

CMake and compile for use on BBB with camera (needs OpenCV)

    cmake -D SIM=false -D USE_CAMERA=true ..

# Running
You can now run the code by executing ./BlueCode .
