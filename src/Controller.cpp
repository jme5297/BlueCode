#include <Controller.h>

using namespace Plant;
using namespace Control;

Controller::Controller(){
	currentVehicleMode = VehicleMode::Track;
	controlMoveIndex = 0;
}
Controller::~Controller(){

}

void Controller::RequestControlMove(ControlMove cm){
	cm.index = controlMoveIndex;
	controlMoveBuffer.push_back(cm);
	return;
}

void Controller::Run(){

	// Account for a breif period where there is nothing occuring
	if(controlMoveBuffer.empty()){ return; }

	// If the buffer is empty, then don't run anything.
	if(controlMoveBuffer[controlMoveIndex].done){
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

	ControlMove move = controlMoveBuffer[controlMoveIndex];
	switch(move.state){
		case ControlState::PayloadDrop:
			PayloadDrop();
			break;
	}

	return;
}

void Controller::PayloadDrop(){
	std::cout << "Performing payload drop... " << std::endl;
	// Payload drop here
	bool complete = false;


	complete = true;
	if(complete){
		controlMoveBuffer[controlMoveIndex].done = true;
		std::cout << "Payload drop complete!" << std::endl;
	}
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
ControlMove Controller::GetCurrentControlMove(){ return controlMoveBuffer[controlMoveIndex]; }
int Controller::GetControlMoveIndex(){ return controlMoveIndex; }
double Controller::GetMotorLSpeed(){ return motorLSpeed; }
double Controller::GetMotorRSpeed(){ return motorRSpeed; }
double Controller::GetWheelSpeed(){ return wheelSpeed; }
double Controller::GetWheelSteering(){ return wheelSteering; }
std::vector<ControlMove> Controller::GetControlMoveBuffer(){ return controlMoveBuffer; }
