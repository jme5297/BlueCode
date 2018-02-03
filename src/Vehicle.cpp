#include <PlantModel/Vehicle.h>

using namespace Plant;

void Vehicle::SetHeading(double val){
  heading = val;
  return;
}
void Vehicle::InitializeGPS(){
  gps.initialized = true;
  return;
}
double Vehicle::GetHeading(){ return heading; }
pGPS& Vehicle::GetGPS(){ return gps; }
pCam& Vehicle::GetCamera(){ return cam; }
pLas& Vehicle::GetLaser(){ return las; }
pMot& Vehicle::GetMotorL(){ return motL; }
pMot& Vehicle::GetMotorR(){ return motR; }
