#include <sensors/Laser/Laser_generic.h>

using namespace sensors;
using namespace Plant;

Laser::Laser(){

}
Laser::~Laser(){

}
bool Laser::Init(){

	return true;
}
bool Laser::Reset(){

	return true;
}
double Laser::ReadLaser(){

	#ifdef SIM // sim mode
	return PlantModel::GetVehicle()->las.val;
	#else
	// ACTUAL laser information here


	return 1.0;
	#endif


}
