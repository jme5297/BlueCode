# BlueCode
Coding base for AERSP 440's Blue Team (a.k.a. the winning team)

# Config & Build
Checkout the master branch or any tag of the code to test to a local directory. Then, modify the following files to your liking:

main.cpp - can control time-step info (like Physics timestep, GNC algorithm clock frequencies, etc).
Vehicle.cpp - can control vehicle properties (sensor locations (eventually), chassis width, etc.) and initial conditions

After sim is modified to your liking, simply build by navigating to the bin/ directory and run the following commands:
make clean
make

You can now run the code by executing ./main .
