#include <PlantModel.h>

using namespace sensors;

// Declare statics
Vehicle PlantModel::veh;

void PlantModel::Initialize(){
  /*
  veh.speed = 0.0;
  veh.heading = 0.0;
  veh.gps = { false, {0.0, 0.0}};
  veh.cam = { false, false };
  veh.las = { false, 0.0 };
  veh.motL = { false, 0.0 };
  veh.motR = { false, 0.0 };
  */
}

void Vehicle::InitializeGPS(){
  gps.initialized = true;
}
Vehicle& PlantModel::GetVehicle(){
  return veh;
}
pGPS& Vehicle::GetGPS(){
  return gps;
}
pCam& Vehicle::GetCamera(){
  return cam;
}
pLas& Vehicle::GetLaser(){
  return las;
}
pMot& Vehicle::GetMotorL(){
  return motL;
}
pMot& Vehicle::GetMotorR(){
  return motR;
}
