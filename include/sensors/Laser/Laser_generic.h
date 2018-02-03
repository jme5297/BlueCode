#pragma once
#include <sensors/Sensors_generic.h>
#include <PlantModel/PlantModel.h>

namespace sensors{
  // Main class for Laser capabilities
  class Laser{

  public:
    Laser();
    ~Laser();
    bool Init();
    bool Reset();
    double ReadLaser();

  };
}
