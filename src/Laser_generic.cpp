#include <sensors/Laser/Laser_generic.h>

using namespace sensors;

#ifdef SIM
using namespace Plant;
#endif

using namespace std;

Laser::Laser(int i) {
	ID = i;
}
Laser::~Laser() {

}
bool Laser::Init() {

#ifdef USE_LASER
	ifstream laser;
	string laserLoc = "/sys/class/gpio/gpio" + std::to_string(ID) + "/value";
	laser.open(laserLoc);
	if (!laser.is_open()) {
		cout << "INIT ERROR: Unable to read the laser data for GPIO " + std::to_string(ID) + ".\n";
		return false;
	}
#endif

	return true;
}
bool Laser::Reset() {

	return true;
}
bool Laser::ReadLaser() {

#ifdef USE_LASER

	// ACTUAL laser information here
	int digit = -1;
	ifstream laser;

	string laserLoc = "/sys/class/gpio/gpio" + std::to_string(ID) + "/value";
	laser.open(laserLoc);

	if (laser.is_open()) {
		while (laser.good()) { laser >> digit; }
	}
	else {
		cout << "Unable to read the laser data\n";
		return false;
	}
	if (digit == 0) {
		// Make sure that we are not testing a laser that isn't working.
		if(ID == Parser::GetTemp_Disable_Laser() || Parser::GetTemp_Disable_Laser() == -1){
			return false;
		}
		return true;
	}
	else if (digit == 1) { return false; }
	return false;

#else

#ifdef SIM
	Vehicle* v = PlantModel::GetVehicle();
	bool val = false;
	int id1 = Parser::GetLaser_Left();
	int id2 = Parser::GetLaser_Right();
	if (ID == id1) {
		val = v->lasers[0].val;
	}
	else {
		val = v->lasers[1].val;
	}
	return val;
#endif
	return false;

#endif

}
