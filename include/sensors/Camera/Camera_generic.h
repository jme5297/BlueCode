#pragma once
#include <sensors/Sensors_generic.h>
#include <PlantModel/PlantModel.h>

#ifdef USE_CAMERA
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#endif

namespace sensors{

  #ifdef USE_CAMERA
  using namespace cv;
  #endif

  // Main class for handling camera capabilities
  class Camera{
  public:
    Camera();
    ~Camera();
    bool Init();
    bool Reset();
    bool Enable();
    bool Disable();
    bool TakeImage(int i);


  };

}
