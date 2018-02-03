#pragma once
#include <cmath>
#include <chrono>
#include <iostream>
#include <PlantModel/Vehicle.h>

namespace Plant{

  class PlantModel{
  public:
    static void Initialize();
    static void Run(double dt);
    static Coordinate GetGPSCoords();
    static Vehicle * GetVehicle();
    static double GetElapsedSeconds();
    static std::chrono::duration<double> GetSimDuration();
    static void PrintStatus();

  protected:
    static std::chrono::time_point<std::chrono::system_clock> initTime;
    static std::chrono::time_point<std::chrono::system_clock> lastRunCall;
    static Vehicle veh;

    // For determining if Run is being called on first pass
    static bool fp_run;
  };

}
