#include <sensors/Laser/Laser_generic.h>

using namespace sensors;

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

	return 1.0;
}
