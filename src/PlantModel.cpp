#define PI 3.14159265
#include <PlantModel/PlantModel.h>
#include <iostream>

using namespace sensors;
using namespace Plant;

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

  double speedL = veh.GetMotorL().val;
  double speedR = veh.GetMotorR().val;

  // Update position based on speed
  double factor = 1.0;
  double x,y;
  Coordinate gpsCoords = veh.GetGPS().coords;
  x = gpsCoords.lon;
  y = gpsCoords.lat;

  double diff = speedL - speedR;

  // Update heading if the vehicle is turning.
  veh.SetHeading( veh.GetHeading() + 10.0*diff );

  double dist = factor*(speedL + speedR)/2.0;

  x += dist*sin(veh.GetHeading() * PI / 180.0);
  y += dist*cos(veh.GetHeading() * PI / 180.0);

  Coordinate updatedCoords = {x, y};
  veh.GetGPS().coords = updatedCoords;

  return;
}
Vehicle& PlantModel::GetVehicle(){
  return veh;
}
