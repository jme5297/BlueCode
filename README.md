# BlueCode
Coding base for AERSP 440's Blue Team (a.k.a. the winning team)

# Prerequisites
* Download and install CMake.
* **MacOSX**: make sure you have the XCode development suite (just the tool library is enough.)
* **Windows**: make sure you have Visual Studio 2015 or higher.
* **Linux**: make sure you have gcc, g++, and gdb.

## Camera Testing & OpenCV
* Windows:
  * https://docs.opencv.org/2.4/doc/tutorials/introduction/windows_install/windows_install.html
* Linux:
  * https://www.learnopencv.com/install-opencv3-on-ubuntu/
* MacOSX:
  * Get Homebrew if you haven't already.
    * `ruby -e "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/master/install)"`
    * `brew update`
  * Get OpenCV from homebrew.
    * `brew tap homebrew/science`
    * `brew install opencv3 --with-contrib`

## Simulation Testing
You can run the simulation with or without the use of a pre-compiled 3d engine called Irrlicht. If the simulation is ran without Irrlicht, then there will be no obstacle detection and no laser sensor readings. Prerequisites for Irrlicht are below.
* For Linux/WSL users:
  * `sudo apt-get -y install build-essential xserver-xorg-dev x11proto-xf86vidmode-dev libxxf86vm-dev mesa-common-dev libgl1-mesa-dev libglu1-mesa-dev libxext-dev libxcursor-dev`

# CMake
An option is to run cmake-gui. You can also control from command line. If you wish to run cmake from the command line, you'll have to set your variables with `-D <variable>=<value>`. Multiple variables can be passed to the command line (example: `cmake -D <var1>=true -D <var2>=true ..`).
* Make a directory called "build" in the main BlueCode directory.
* Move into the "build" directory.

## CMake Method (GUI or Command-Line)
* **CMake GUI**
  * Source directory is "BlueCode".
  * Build directory is "BlueCode/build".
  * Hit "Configure".
  * Check and uncheck any parameters that you wish.
  * Hit "Configure" again, until no more parameters are highlighted red.
  * Hit "Generate".
* **CMake Command-Line**
  * `cmake -D <var1>=<value1> -D <var2>=<value2> ..`

## CMake Parameters
| Parameter/Variable | Description | Default Value |
| --- | --- | --- |
| `SIM`     | Uses a physics plant model and fake motor controls. | `false` |
| `DEBUG`   | Uses a counter loop for time rather than std::chrono. <br> Do NOT use `DEBUG` when running on BBB. | `false` |
| `USE_IRRLICHT`     | Render a 3D engine of the simulation and calculate obstacles & laser ranges. <br> Note: `SIM` is required. | `false` |
| `USE_CAMERA` | Use whatever camera is attached to device to take images. <br> Note: requires OpenCV. | `false` |


# Build
* **Windows**
  * Open up the BlueCode.sln file that was created using Visual Studio.
  * Compile to Win32 (either Debug or Release).
  * Change the startup project to BlueCode (right-click on the BlueCode project --> Set as Startup Project)
  * Hit run, or navigate to the build directory, open a terminal window, and type `./Debug/BlueCode.exe` or `./Release/BlueCode.exe`.
* **Linux/MacOSX**
  * Run `make` in the "BlueCode/build" folder.

# Running
* **Windows**
  * Option 1: In Visual Studio, hit run.
  * Option 2: In Windows Command Prompt or PowerShell, navigate to the build directory, type `./Debug/BlueCode.exe` or `./Release/BlueCode.exe`.
* **Linux/MacOSX**
  * Go to the "BlueCode/build" directory and run `./BlueCode` .
