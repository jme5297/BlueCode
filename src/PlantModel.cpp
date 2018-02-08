#define PI 3.14159265
#include <PlantModel/PlantModel.h>
#include <iostream>
#include <string>

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

  double dx;
  double dy;

  switch(GetVehicle()->vehicleType){
    //-------------------------
    //    TRACK MODE 
    //-------------------------
    case VehicleType::Track:{
      double speedL = GetVehicle()->motL.val * GetVehicle()->maxSpeedMPS;
      double speedR = GetVehicle()->motR.val * GetVehicle()->maxSpeedMPS;
      // Angle is in RADIANS
      double dtheta = (speedL - speedR) / GetVehicle()->width * dt;
      // Update heading if the vehicle is turning.
      GetVehicle()->heading += dtheta * 180.0 / PI ;
      GetVehicle()->heading = fmod(GetVehicle()->heading, 360.0);
      
      // Turning radius of the INSIDE track.
      double r;
      if(dtheta <= 1e-15){
        r = -1.0; // turning radius is infinity here.
        dx = speedL*dt*sin(GetVehicle()->heading * PI / 180.0);
        dy = speedL*dt*cos(GetVehicle()->heading * PI / 180.0);
      }else{
        r = speedR * dt / dtheta;
        dx = (r + GetVehicle()->width/2.0)*dtheta*sin(GetVehicle()->heading * PI / 180.0);
        dy = (r + GetVehicle()->width/2.0)*dtheta*cos(GetVehicle()->heading * PI / 180.0);
      }
      break;
    }
    //-------------------------
    //    WHEEL MODE 
    //-------------------------
    case VehicleType::Wheel:
      // http://www.davdata.nl/math/turning_radius.html
      // This is the dav-data model for a bicycle, but it should work well enough for us.
      // NOTE: This is ideal for the turning raidus of a bicycle. There may have to be a more
      // complex model used if this is not sufficient.
      double w  = GetVehicle()->length;
      double alpha = GetVehicle()->wheelSteeringN * GetVehicle()->maxWheelSteeringAngleDeg;
      double speed = GetVehicle()->wheelSpeedN * GetVehicle()->maxSpeedMPS;

      if(fabs(alpha) <= 1e-15){
        dx = speed*dt*sin(GetVehicle()->heading * PI / 180.0);
        dy = speed*dt*cos(GetVehicle()->heading * PI / 180.0);
      }else{
        double R = w / sin(fabs(alpha) * PI / 180.0);
        double r = w / tan(fabs(alpha) * PI / 180.0);
        // Calculate the approximate centroid radius.
        double wid = GetVehicle()->width;
        double R_cent = 0.5*(0.5*(R+R+wid) + 0.5*(r+r+wid));
        double dtheta = speed * dt / R_cent; // radians

        if(alpha < 0.0){
          GetVehicle()->heading -= dtheta * 180.0 / PI ;
        }else{
          GetVehicle()->heading += dtheta * 180.0 / PI ;
        }
        GetVehicle()->heading = fmod(GetVehicle()->heading, 360.0);

        dx = (R_cent)*dtheta*sin(GetVehicle()->heading * PI / 180.0);
        dy = (R_cent)*dtheta*cos(GetVehicle()->heading * PI / 180.0);
      }
      break;
  }

  // Update the vehicle position.
  GetVehicle()->gps.coords.lon += dx;
  GetVehicle()->gps.coords.lat += dy;

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
