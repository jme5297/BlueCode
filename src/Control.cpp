#include <Control.h>

using namespace Plant;
using namespace Control;

Controller::Controller(){
	currentVehicleMode = VehicleMode::Track;

}
Controller::~Controller(){

}

void Controller::Run(){

	return;
}

void Controller::PayloadDrop(){

	return;
}

void Controller::SetWheelSpeed(double s){
	wheelSpeed = s;

	#ifdef SIM
	PlantModel::GetVehicle()->wheelSpeed = s;
	#else
	// Actual speed control goes here
	#endif
}
void Controller::SetWheelSteering(double s){
	wheelSteering = s;

	#ifdef SIM
	PlantModel::GetVehicle()->wheelSteering = s;
	#else
	// Actual steering control goes here
	#endif
}

void Controller::SetMotorSpeeds(double speed){

	motorLSpeed = speed;
	motorRSpeed = speed;

	#ifdef SIM
	PlantModel::GetVehicle()->motL.val = speed;
	PlantModel::GetVehicle()->motR.val = speed;
	#else
	// Actual motor control goes here
	#endif

	return;
}
void Controller::SetMotorLSpeed(double speed){

	motorLSpeed = speed;

	#ifdef SIM
	PlantModel::GetVehicle()->motL.val = speed;
	#else
	// Actual motor control goes here
	#endif

	return;
}
void Controller::SetMotorRSpeed(double speed){

	motorRSpeed = speed;

	#ifdef SIM
	PlantModel::GetVehicle()->motR.val = speed;
	#else
	// Actual motor control goes here
	#endif

	return;
}
double Controller::GetMotorLSpeed(){ return motorLSpeed; }
double Controller::GetMotorRSpeed(){ return motorRSpeed; }
double Controller::GetWheelSpeed(){ return wheelSpeed; }
double Controller::GetWheelSteering(){ return wheelSteering; }
