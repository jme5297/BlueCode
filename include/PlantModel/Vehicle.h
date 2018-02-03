#pragma once
#include <sensors/Sensors_generic.h>
#include <cmath>

using namespace sensors;

namespace Plant{

  enum VehicleType{
    Track,
    Wheel
  };

  struct pGPS{
  public:
    bool initialized;
    Coordinate coords;
  };
  struct pCam{
  public:
    bool initialized;
    bool enabled;
  };
  struct pLas{
  public:
    bool initialized;
    double val;
  };
  struct pMot{
  public:
    bool initialized;
    double val;
  };

  class Vehicle{
  public:
    Vehicle();
    ~Vehicle();
    void InitializeGPS();
    double width;
    double heading;
    pGPS gps;
    pCam cam;
    pLas las;
    pMot motL;
    pMot motR;
    VehicleType vehicleType;
    double wheelSpeed;
    double wheelHeading;
  };
}
