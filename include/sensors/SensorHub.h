#pragma once
#include <sensors/Sensors_generic.h>
#include <sensors/GPS/GPS_generic.h>
#include <sensors/Laser/Laser_generic.h>
#include <sensors/Camera/Camera_generic.h>

/// All sensor-related classes and members
namespace sensors{

  class SensorHub{
  public:
    SensorHub();
    ~SensorHub();

    bool InitAllSensors();
    bool ResetAllSensors();

    GPS& GetGPS();
    std::vector<Laser>& GetLasers();
    Camera& GetCamera();

  protected:
    GPS gps0;
    std::vector<Laser> lasers;
    Camera cam0;

  };
}
