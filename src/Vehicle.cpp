#include <PlantModel/Vehicle.h>
#include <iostream>

using namespace Plant;

Vehicle::Vehicle(){

  vehicleType = VehicleType::Track;
  
  // Do more initialization here
  width = 0.2;

}
Vehicle::~Vehicle(){

}

void Vehicle::InitializeGPS(){
  gps.initialized = true;
  gps.coords.lat = 0.0;
  gps.coords.lon = 0.0;
  return;
}
