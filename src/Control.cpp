#include <Control.h>

using namespace Plant;
using namespace Guidance;
using namespace Control;

Controller::Controller(){
	currentVehicleMode = VehicleMode::Track;

}
Controller::~Controller(){

}

void Controller::Run(Guider& g){

	// Account for a breif period where there is nothing occuring
	if(g.GetGuidanceManeuverBuffer().empty()){ return; }

	// If the buffer is empty, then don't run anything.
	if(g.GetCurrentGuidanceManeuver().done){
		switch(currentVehicleMode){
			case VehicleMode::Wheel:
				SetWheelSpeed(0.0);
				SetWheelSteering(0.0);
				break;
			case VehicleMode::Track:
				SetMotorSpeeds(0.0);
				break;
		}
		return;
	}

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
