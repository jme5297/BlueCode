#pragma once
#include <sensors/Sensors_generic.h>

#ifdef SIM
#include <PlantModel/PlantModel.h>
#endif

namespace sensors{
  // Main class for Laser capabilities
  class Laser{

  public:
    Laser(int);
    ~Laser();
    bool Init();
    bool Reset();
    bool ReadLaser();  ///< Read value coming from a specific laser.


  protected:
    int ID;

  };
}
