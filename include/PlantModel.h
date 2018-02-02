#pragma once
#include <sensors/Sensors_generic.h>
#include <cmath>

using namespace sensors;

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
  friend class PlantModel;
public:
  void InitializeGPS();
  pGPS& GetGPS();
  pCam& GetCamera();
  pLas& GetLaser();
  pMot& GetMotorL();
  pMot& GetMotorR();
  void SetHeading(double val);
protected:
  double heading;
  pGPS gps;
  pCam cam;
  pLas las;
  pMot motL;
  pMot motR;
};

class PlantModel{
public:
  static void Initialize();
  static void Run();
  static Coordinate GetGPSCoords();
  static Vehicle& GetVehicle();

protected:
  static Vehicle veh;
};
