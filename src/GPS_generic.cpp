#include <preprocdef.h>
#include <sensors/GPS/GPS_generic.h>

using namespace sensors;

GPS::GPS(){

}
GPS::~GPS(){

}
bool GPS::Init(){

	#ifdef SIM
	PlantModel::GetVehicle().InitializeGPS();
	return PlantModel::GetVehicle().GetGPS().initialized;
	#else

	#endif

	return true;
}
bool GPS::Reset(){

	return true;
}
Coordinate GPS::GetCurrentGPSCoordinates(){

	#ifdef SIM
	return PlantModel::GetVehicle().GetGPS().coords;
	#else

	#endif
}
