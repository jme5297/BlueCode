#include <PlantModel/Vehicle.h>
#include <iostream>

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
	// Initialization for wheel model
	maxWheelSteeringAngleDeg = 15.0;
}
Vehicle::~Vehicle() {

}
void Vehicle::InitializeGPS() {
	gps.initialized = true;
	gps.coords.lat = 0.0;
	gps.coords.lon = 0.0;
	return;
}
