#pragma once
#include <sensors/Sensors_generic.h>
#include <cmath>

using namespace sensors;

namespace Plant{

  struct pGPS{
    bool initialized;
    Coordinate coords;
  };
  struct pCam{
    bool initialized;
    bool enabled;
  };
  struct pLas{
    bool initialized;
    double val;
  };
  struct pMot{
    bool initialized;
    double val;
  };

  class Vehicle{
  public:
    void InitializeGPS();
    pGPS& GetGPS();
    pCam& GetCamera();
    pLas& GetLaser();
    pMot& GetMotorL();
    pMot& GetMotorR();
    void SetHeading(double val);
    double GetHeading();
  protected:
    double heading;
    pGPS gps;
    pCam cam;
    pLas las;
    pMot motL;
    pMot motR;
  };
}
