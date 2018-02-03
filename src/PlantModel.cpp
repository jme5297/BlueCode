#define PI 3.14159265
#include <PlantModel/PlantModel.h>
#include <iostream>

using namespace sensors;
using namespace Plant;

// Declare statics
Vehicle PlantModel::veh;
std::chrono::time_point<std::chrono::system_clock> PlantModel::initTime;
std::chrono::time_point<std::chrono::system_clock> PlantModel::lastRunCall;
bool PlantModel::fp_run;

void PlantModel::Initialize(){
  std::cout << "PLANT INITIALIZED " << GetVehicle()->gps.coords.lon << "\n\n";
  initTime = std::chrono::system_clock::now();
  fp_run = true;
}

// Really, really simple physics model.
void PlantModel::Run(double dt){

  switch(GetVehicle()->vehicleType){
    case VehicleType::Track:{
      double speedL = GetVehicle()->motL.val;
      double speedR = GetVehicle()->motR.val;
      // Angle is in RADIANS
      double dtheta = (speedL - speedR) / GetVehicle()->width * dt;
      // Update heading if the vehicle is turning.
      GetVehicle()->heading += dtheta * 180.0 / PI ;
      GetVehicle()->heading = fmod(GetVehicle()->heading, 360.0);
      // Turning radius
      double r;
      double dx;
      double dy;
      if(dtheta <= 1e-15){
        r = -1.0; // turning radius is infinity here.
        dx = speedL*dt*sin(GetVehicle()->heading * PI / 180.0);
        dy = speedL*dt*cos(GetVehicle()->heading * PI / 180.0);
      }else{
        r = speedR * dt / dtheta;
        dx = (r + GetVehicle()->width/2.0)*dtheta*sin(GetVehicle()->heading * PI / 180.0);
        dy = (r + GetVehicle()->width/2.0)*dtheta*cos(GetVehicle()->heading * PI / 180.0);
      }
      GetVehicle()->gps.coords.lon += dx;
      GetVehicle()->gps.coords.lat += dy;
      break;
    }
    case VehicleType::Wheel:

      break;
  }

  return;
}
void PlantModel::PrintStatus(){
  std::cout << "t: " << GetElapsedSeconds() <<
    " --- lat: " << std::to_string(GetVehicle()->gps.coords.lat) << ", lon: " << GetVehicle()->gps.coords.lon << "\n";
}
std::chrono::duration<double> PlantModel::GetSimDuration(){
  return std::chrono::system_clock::now()-initTime;
}
double PlantModel::GetElapsedSeconds(){
  std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now()-initTime;
  return elapsed_seconds.count();
}

Vehicle * PlantModel::GetVehicle(){
  return &veh;
}
