#include <sensors/GPS/GPS_generic.h>

using namespace sensors;
using namespace Times;

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
	currentGPSCoordinates = PlantModel::GetVehicle()->gps.coords;
	lastCoordinates = currentGPSCoordinates;
	TimeModule::InitProccessCounter("GPS", Parser::GetRefresh_GPS());
	vehicleHeading = 0.0;
	return PlantModel::GetVehicle()->gps.initialized;
	#else
	// Actual GPS initialization code goes here

	// THIS IS TEMPORARY
	Coordinate c;
	c.lat = Parser::GetInitialLatitude();
	c.lon = Parser::GetInitialLongitude();
	currentGPSCoordinates = c;
	lastCoordinates = c;
	vehicleHeading = Parser::GetInitialHeading();

	#endif

	return true;
}
void GPS::Run(){

	double PI = 3.14159265;

	#ifdef SIM
	if (TimeModule::ProccessUpdate("GPS")) {
		// Calculate GPS.
		Coordinate c = PlantModel::GetVehicle()->gps.coords;
		c.lat += (-0.5 + ((double)rand() / (RAND_MAX))) * gpsUncertainty / latToM;
		c.lon += (-0.5 + ((double)rand() / (RAND_MAX))) * gpsUncertainty / lonToM;
		currentGPSCoordinates = c;
		// Calculate Ground Course (heading).
		Coordinate c1 = lastCoordinates;
		Coordinate c2 = PlantModel::GetVehicle()->gps.coords;
		double dx = c2.lon - c1.lon;
		double dy = c2.lat - c1.lat;
		double z = atan2(dy, dx) * 180.0 / PI;
		double head = 90.0 - z;
		head = (head < 0.0) ? 360.0 + head : head;
		vehicleHeading = head + (-0.5 + ((double)rand() / (RAND_MAX))) * Parser::GetGPSHeadingUncertainty();
		lastCoordinates = PlantModel::GetVehicle()->gps.coords;
	}
	#else

	#endif
}
bool GPS::Reset(){

	return true;
}
Coordinate GPS::GetCurrentGPSCoordinates(){

	return currentGPSCoordinates;
}
double GPS::GetGPSGroundCourse(){

	return vehicleHeading;
}
