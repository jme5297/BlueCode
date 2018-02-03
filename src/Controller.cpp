#include <Controller.h>

using namespace Plant;

Controller::Controller(){

}
Controller::~Controller(){

}

void Controller::SetMotorSpeeds(double speed){

	motorLSpeed = speed;
	motorRSpeed = speed;

	#ifdef SIM
	PlantModel::GetVehicle().GetMotorL().val = speed;
	PlantModel::GetVehicle().GetMotorR().val = speed;
	#else

	// Actual motor control goes here

	#endif

	return;
}
void Controller::SetMotorLSpeed(double speed){

	motorLSpeed = speed;

	#ifdef SIM
	PlantModel::GetVehicle().GetMotorL().val = speed;
	#else

	// Actual motor control goes here

	#endif

	return;
}
void Controller::SetMotorRSpeed(double speed){

	motorRSpeed = speed;

	#ifdef SIM
	PlantModel::GetVehicle().GetMotorR().val = speed;
	#else

	// Actual motor control goes here

	#endif

	return;
}
double Controller::GetMotorLSpeed(){
	return motorLSpeed;
}
double Controller::GetMotorRSpeed(){
	return motorRSpeed;
}
