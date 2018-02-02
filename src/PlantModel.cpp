#define PI 3.14159265
#include <PlantModel.h>
#include <iostream>

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

void PlantModel::Run(){
  
  // Really, really simple physics model.

  double speedL = veh.motL.val;
  double speedR = veh.motR.val;

  // Update position based on speed
  double factor = 1.0;
  double x,y;
  Coordinate gpsCoords = veh.gps.coords;
  x = gpsCoords.lon;
  y = gpsCoords.lat;

  double diff = speedL - speedR;
 
  // Update heading if the vehicle is turning.
  veh.heading += 10.0*diff;

  double dist = factor*(speedL + speedR)/2.0;

  x += dist*sin(veh.heading * PI / 180.0);
  y += dist*cos(veh.heading * PI / 180.0);

  Coordinate updatedCoords = {x, y};
  veh.gps.coords = updatedCoords;

  return;
}

void Vehicle::SetHeading(double val){
  heading = val;
  return;
}
void Vehicle::InitializeGPS(){
  gps.initialized = true;
  return;
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
