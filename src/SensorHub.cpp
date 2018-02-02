#include <sensors/SensorHub.h>

using namespace sensors;

SensorHub::SensorHub(){

}
SensorHub::~SensorHub(){

}
bool SensorHub::InitAllSensors(){

  bool gpsi = gps0.Init();
  bool lasi = las0.Init();
  bool cami = cam0.Init();

  if(gpsi && lasi && cami){
    return true;
  }

  return false;
}
bool SensorHub::ResetAllSensors(){

  bool gpsr = gps0.Reset();
  bool lasr = las0.Reset();
  bool camr = cam0.Reset();

  if(gpsr && lasr && camr){
    return true;
  }

  return false;
}
GPS& SensorHub::GetGPS(){ return gps0; }
Laser& SensorHub::GetLaser(){ return las0; }
Camera& SensorHub::GetCamera(){ return cam0; }
