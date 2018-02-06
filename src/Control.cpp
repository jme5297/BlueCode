#include <Control.h>

using namespace Plant;
using namespace Guidance;
using namespace Control;

Controller::Controller(){
	currentVehicleMode = VehicleMode::Track;

}

void Controller::SetCurrentVehicleMode(VehicleMode vm)
{
	currentVehicleMode = vm;
}
VehicleMode Controller::GetCurrentVehicleMode()
{	
	return currentVehicleMode;
}

void Controller::Run(Guider& g){

	// If the buffer is empty, then don't run anything.
	if(g.GetGuidanceManeuverBuffer().empty() || g.GetCurrentGuidanceManeuver().done){
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
	else{
		switch(g.GetCurrentGuidanceManeuver().state){
			case ManeuverState::Calibrate:

				break;
			case ManeuverState::Turn:
				switch(currentVehicleMode){
					case VehicleMode::Wheel:
						SetWheelSpeed(g.GetCurrentGuidanceManeuver().speed);
						SetWheelSteering((double)g.GetCurrentGuidanceManeuver().turnDirection);
						break;
					case VehicleMode::Track:
						if(g.GetCurrentGuidanceManeuver().turnDirection == -1){
							SetMotorLSpeed(g.GetCurrentGuidanceManeuver().speed*0.8);
							SetMotorRSpeed(g.GetCurrentGuidanceManeuver().speed);
						}else{
							SetMotorLSpeed(g.GetCurrentGuidanceManeuver().speed);
							SetMotorRSpeed(g.GetCurrentGuidanceManeuver().speed*0.8);
						}
						break;
				}
				break;
			case ManeuverState::Maintain:
				switch(currentVehicleMode){
					case VehicleMode::Wheel:
						SetWheelSpeed(g.GetCurrentGuidanceManeuver().speed);
						SetWheelSteering(0.0);
						break;
					case VehicleMode::Track:
						SetMotorSpeeds(1.0);
						break;
				}
				break;
			case ManeuverState::AvoidDiverge:

				break;
			case ManeuverState::AvoidConverge:

				break;
			case ManeuverState::PayloadDrop:

				break;
			case ManeuverState::Complete:

				break;
		}
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
