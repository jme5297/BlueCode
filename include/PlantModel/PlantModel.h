#pragma once
#include <cmath>
#include <chrono>
#include <iostream>
#include <PlantModel/Vehicle.h>

namespace Plant{

  class PlantModel{
  public:
    static void Initialize();

    /** 
     * \todo The plant model has not yet been checked to validate backward steering speeds
     * for both wheel vehicles and track vehicles. Ideally, track vehicles should be able to
     * pivot around the centroid. This capability should be built in.
     */
    static void Run(double dt);         ///< Main execution function for the plant model.
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
