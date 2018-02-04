#include <Guidance.h>

using namespace Guidance;
using namespace Control;

Guider::Guider(){
  GuidanceManeuverIndex = 0;
}

void Guider::RequestGuidanceManeuver(GuidanceManeuver cm){
	cm.index = GuidanceManeuverIndex;
	GuidanceManeuverBuffer.push_back(cm);
	return;
}

void Guider::Run(Controller& c){

  // Get the vector of control moves
	std::vector<GuidanceManeuver> buf  = GetGuidanceManeuverBuffer();

  // Account for a breif period where there is nothing occuring
	if(GuidanceManeuverBuffer.empty()){ return; }

	// If the buffer is empty, then don't run anything.
	if(GuidanceManeuverBuffer[GuidanceManeuverIndex].done){
		switch(c.currentVehicleMode){
			case VehicleMode::Wheel:
				c.SetWheelSpeed(0.0);
				c.SetWheelSteering(0.0);
				break;
			case VehicleMode::Track:
				c.SetMotorSpeeds(0.0);
				break;
		}
		return;
	}

	GuidanceManeuver move = GuidanceManeuverBuffer[GuidanceManeuverIndex];
	switch(move.state){
		case ManeuverState::Calibrate:

			break;
		case ManeuverState::Maintain:

			break;
		case ManeuverState::AvoidDiverge:

			break;
		case ManeuverState::AvoidConverge:

			break;
		case ManeuverState::PayloadDrop:
			PayloadDrop(c);
			break;
		case ManeuverState::Complete:

			break;
	}

  return;
}

void Guider::PayloadDrop(Controller& c){
	std::cout << "Performing payload drop... " << std::endl;
	// Payload drop here
	bool complete = false;
	c.PayloadDrop();
	complete = true;
	if(complete){
		GuidanceManeuverBuffer[GuidanceManeuverIndex].done = true;
		std::cout << "Payload drop complete!" << std::endl;
	}
	return;
}

std::vector<GuidanceManeuver> Guider::GetGuidanceManeuverBuffer(){ return GuidanceManeuverBuffer; }
GuidanceManeuver Guider::GetCurrentGuidanceManeuver(){ return GuidanceManeuverBuffer[GuidanceManeuverIndex]; }
int Guider::GetGuidanceManeuverIndex(){ return GuidanceManeuverIndex; }
