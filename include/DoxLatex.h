/*! \mainpage Introduction
 *
 * This document outlines the software coding algorithms and structures for the Blue Team.
 *
 * \section s1 Members
 * Eric Burkhart - epb510 \n Anthony Maenza - axm5730 \n Arthur Cruz - amc663 \n
 * Jason Everett - jme5297 \n Mohamed Abuzaid - mza5399 \n Wei-cheng Hsu - wfh5069 \n
 * Zouzhou Chen - zfc5022 \n
 *
 * \section s2 Code
 * The code for this project is hosted on a private GitHub server. Please contact
 * Jason Everett (jme5297) if you wish to have access to the code.
 * \subsection ss1 Compiling and Running the Program
 * The GitHub repository contains source code for a very simple plant model that can be used to test
 * the code without installing the code onto a BeagleBone board. This plant model can be turned on by
 * making sure that the "#define SIM" line in preprocdef.h is not commented out. When SIM mode is active,
 * all sensors and actuators control a simulated vehicle rather than a real, physical vehicle.
 * \subsection ss2 Debugging the Code
 * The majority of the code, when running on a BeagleBone board, uses the c++ <chrono> steady_clock. If debugging
 * the code is a requirement, then a special mode of simulation that is not steady_clock dependent can be compiled
 * by ensuring that the "#define DEBUG" line in preprocdef.h is not commented.
 *
 * \section s3 Navigating This Document
 * This document is meant to be a reference to anyone who wants to get a feel for the general architecture of the code.
 * There are heirarchy-based diagrams that are created using Graphviz that enable the reader to determine where functions
 * are being called, and what functions are calling a specific function. The digital PDF version of this document
 * contains hyperlinks that allow a reader to quickly navigate to another section of interest.
 *
 * **The main execution file can be found here:** \ref main.cpp
 */
