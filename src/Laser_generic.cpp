#include <sensors/Laser/Laser_generic.h>

using namespace sensors;

#ifdef SIM
using namespace Plant;
#endif

Laser::Laser(int i){
	ID = i;
}
Laser::~Laser(){

}
bool Laser::Init(){

	return true;
}
bool Laser::Reset(){

	return true;
}
bool Laser::ReadLaser(){

	#ifdef SIM // sim mode
	return PlantModel::GetVehicle()->lasers[ID].val;
	#else
	// ACTUAL laser information here


	return 1.0;
	#endif
}
