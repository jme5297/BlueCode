#include <sensors/SensorHub.h>

using namespace sensors;

SensorHub::SensorHub(){
  lasers = {Laser(0)};
}
SensorHub::~SensorHub(){

}
bool SensorHub::InitAllSensors(){

  bool gpsi = gps0.Init();
  bool cami = cam0.Init();

  if(gpsi && cami){
    return true;
  }

  return false;
}
bool SensorHub::ResetAllSensors(){

  bool gpsr = gps0.Reset();
  bool camr = cam0.Reset();

  if(gpsr && camr){
    return true;
  }

  return false;
}
GPS& SensorHub::GetGPS(){ return gps0; }
std::vector<Laser>& SensorHub::GetLasers(){ return lasers; }
Camera& SensorHub::GetCamera(){ return cam0; }
