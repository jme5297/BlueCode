#include <sensors/SensorHub.h>

using namespace sensors;

SensorHub::SensorHub() {

	// Initialize all of the class objects.
	cam0 = Camera();
	gps0 = GPS();

	// Initialize two lasers.
	lasers = {
		Laser(Parser::GetLaser_Left()), // Laser 0 represents the left laser.
		Laser(Parser::GetLaser_Center()), // Laser 1 represents the center laser
		Laser(Parser::GetLaser_Right())  // Laser 2 represents the right laser.
	};
}

bool SensorHub::InitAllSensors() {

	// Run initialization routines for all Sensor objects.
	bool gpsi = gps0.Init();
	bool cami = cam0.Init();
	bool las0i = lasers[0].Init();
	bool las1i = lasers[1].Init();

	// If all sensors initialized properly, return true.
	if (gpsi && cami && las0i && las1i) {
		return true;
	}

	return false;
}

bool SensorHub::ResetAllSensors() {

	// Run reset routines for all sensors.
	bool gpsr = gps0.Reset();
	bool camr = cam0.Reset();
	bool las0r = lasers[0].Reset();
	bool las1r = lasers[1].Reset();

	// If sensors reset properly, return true.
	if (gpsr && camr && las0r && las1r) {
		return true;
	}

	return false;
}
