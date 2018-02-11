#pragma once
#include <sensors/Sensors_generic.h>

#ifdef SIM
#include <PlantModel/PlantModel.h>
#endif

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
