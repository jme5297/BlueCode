#define PI 3.14159265
#define PLDIST 5.0

#include <Guidance.h>

using namespace Navigation;
using namespace Guidance;
//using namespace Control;

Guider::Guider(){
  GuidanceManeuverIndex = 0;
  isNavPlanComplete = false;
	coordinateIndex = 0;
}

void Guider::RequestGuidanceManeuver(GuidanceManeuver cm){
	cm.index = GuidanceManeuverIndex;
	GuidanceManeuverBuffer.push_back(cm);
	return;
}

void Guider::Run(Navigator& n){

  // Get the vector of control moves
	std::vector<GuidanceManeuver> buf  = GetGuidanceManeuverBuffer();

  // Determine if we're ready to drop a payload.
	if(n.DistanceBetweenCoordinates(n.GetCoordinates(), n.GetNavPlan().coordinates[coordinateIndex]) <= PLDIST){
		std::cout << "Payload drop time!" << std::endl;
		//Push back a control move to drop payload.
		GuidanceManeuver cm;
		cm.state = ManeuverState::PayloadDrop;
		RequestGuidanceManeuver(cm);

		coordinateIndex++;
		// If this is the last coordinate of the nav plan, then let's wrap it up here.
		if(coordinateIndex == n.GetNavPlan().coordinates.size()){
			isNavPlanComplete = true;
		}
		return;
	}

  if(GuidanceManeuverBuffer.empty() || GuidanceManeuverBuffer[GuidanceManeuverIndex].done){ return; }

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
			PayloadDrop();
			break;
		case ManeuverState::Complete:

			break;
	}

  return;
}

void Guider::PayloadDrop(){
	std::cout << "Performing payload drop... " << std::endl;
	// Payload drop here
	bool complete = false;
	// c.PayloadDrop();
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
bool Guider::IsNavPlanComplete() { return isNavPlanComplete; }
