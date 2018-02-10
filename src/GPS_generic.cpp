#include <sensors/GPS/GPS_generic.h>

using namespace sensors;

#ifdef SIM
using namespace Plant;
#endif

GPS::GPS(){

}
GPS::~GPS(){

}
bool GPS::Init(){

	#ifdef SIM
	PlantModel::GetVehicle()->InitializeGPS();
	return PlantModel::GetVehicle()->gps.initialized;
	#else
	// Actual GPS initialization code goes here
	#endif

	return true;
}
bool GPS::Reset(){

	return true;
}
Coordinate GPS::GetCurrentGPSCoordinates(){

	#ifdef SIM
	return PlantModel::GetVehicle()->gps.coords;
	#else
	// Actual code goes here to get GPS coordinates
	return {0.0, 0.0};
	#endif
}
