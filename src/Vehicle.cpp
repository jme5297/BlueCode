#include <PlantModel/Vehicle.h>

using namespace Plant;

Vehicle::Vehicle() {
	
}
void Vehicle::Initialize() {
	vehicleType = Parser::GetVehicleTypeSim();
	width = Parser::GetVehicleWidth();
	length = Parser::GetVehicleLength();
	height = Parser::GetVehicleHeight();
	heading = Parser::GetInitialHeading();
	maxSpeedMPS = Parser::GetMaxSpeedMPS();
	lasers = Parser::GetPLasers();
	maxWheelSteeringAngleDeg = Parser::GetMaxWheelAngleDegrees();
}
Vehicle::~Vehicle() {

}
void Vehicle::InitializeGPS() {
	gps.initialized = true;
	gps.coords.lat = Parser::GetInitialLatitude();
	gps.coords.lon = Parser::GetInitialLongitude();
	return;
}
