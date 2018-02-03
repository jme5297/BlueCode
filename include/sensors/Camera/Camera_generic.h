#pragma once
#include <sensors/Sensors_generic.h>
#include <PlantModel/PlantModel.h>

namespace sensors{

  // Main class for handling camera capabilities
  class Camera{
  public:
    Camera();
    ~Camera();
    bool Init();
    bool Reset();
    bool Enable();
    bool Disable();
    bool TakeImage();


  };

}
