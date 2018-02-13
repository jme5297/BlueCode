#include <sensors/GPS/GPS_generic.h>

using namespace sensors;

#ifdef SIM
using namespace Plant;
#endif

GPS::GPS(){
#ifdef SIM
	srand(time(0));
#endif
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
	Coordinate c = PlantModel::GetVehicle()->gps.coords;
	c.lat += (-0.5 + ((double)rand() / (RAND_MAX))) * gpsUncertainty / latToM;
	c.lon += (-0.5 + ((double)rand() / (RAND_MAX))) * gpsUncertainty / lonToM;
	return c;
	#else
	// Actual code goes here to get GPS coordinates
	return {0.0, 0.0};
	#endif
}
