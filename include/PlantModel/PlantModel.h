#pragma once
#include <cmath>
#include <PlantModel/Vehicle.h>

namespace Plant{

  class PlantModel{
  public:
    static void Initialize();
    static void Run();
    static Coordinate GetGPSCoords();
    static Vehicle& GetVehicle();

  protected:
    static Vehicle veh;
  };

}
