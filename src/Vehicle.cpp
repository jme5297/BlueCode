#include <PlantModel/Vehicle.h>
#include <iostream>

using namespace Plant;

Vehicle::Vehicle(){

  vehicleType = VehicleType::Wheel;
  
  // Do more initialization here
  width = 0.2;
  length = 0.5;
  maxSpeedMPS = 1.0;

  // Initialization for wheel model
  maxWheelSteeringAngleDeg = 30.0;

}
Vehicle::~Vehicle(){

}

void Vehicle::InitializeGPS(){
  gps.initialized = true;
  gps.coords.lat = 0.0;
  gps.coords.lon = 0.0;
  return;
}
