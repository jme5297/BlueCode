#define PI 3.14159265
#define PLDIST 2.0

#include <Guidance.h>

using namespace Navigation;
using namespace Guidance;

/**
 * Sets a few protected members.
 * @param[out] GuidanceManeuverIndex
 * @param[out] isNavPlanComplete
 * @param[out] coordinateIndex
 */
Guider::Guider(){
  GuidanceManeuverIndex = -1;
  isNavPlanComplete = false;
	coordinateIndex = 0;
}

/**
 * This will add a guidance maneuver to the end of the guidance maneuver buffer.
 * @param[in]   gm        - A pre-constructed guidance maneuver
 * @param[out]  gm.index  - Set the index of the guidance maneuver in the buffer.
 */
void Guider::RequestGuidanceManeuver(GuidanceManeuver gm){
  GuidanceManeuverIndex++;
	gm.index = GuidanceManeuverIndex;
	GuidanceManeuverBuffer.push_back(gm);
	return;
}

/**
 * Controls the main operations of the Guider class. This can be separated into three different
 * sections:
 * - **Calculations**: Perform any guidance-specific calculations to be used later in the function.
 * - **Analyze**: Determine based on navigation information if new guidance maneuvers should be added.
 * - **Perform**: Perform actions based on the current guidance maneuver from the buffer.
 * @param[in]   n     - Reference to the navigator.
 * @param[out]  GuidanceManeuverBuffer
 */
void Guider::Run(Navigator& n){

  Movement m;
  m = n.CalculateMovement(n.GetCoordinates(), n.GetNavPlan().coordinates[coordinateIndex]);

  double x1 = sin(n.GetHeading() * PI / 180.0);
  double y1 = cos(n.GetHeading() * PI / 180.0);
  double x2 = sin(m.heading * PI / 180.0);
  double y2 = cos(m.heading * PI / 180.0);

  double offAngle;
  double dott = x1*x2 + y1*y2;      // dot product between [x1, y1] and [x2, y2]
  double det = x1*y2 - y1*x2;      // determinant
  offAngle = atan2(det, dott) * 180.0 / PI;  // atan2(y, x) or atan2(sin, cos)

  /// @todo Determine if there is a better way to organize this sequence of events.

  // Determine if we're ready to drop a payload.
	if(n.DistanceBetweenCoordinates(n.GetCoordinates(), n.GetNavPlan().coordinates[coordinateIndex]) <= PLDIST
    &&  !(
        GuidanceManeuverBuffer[GuidanceManeuverIndex].done == false &&
        GuidanceManeuverBuffer[GuidanceManeuverIndex].state == ManeuverState::PayloadDrop
        )
    ){
    std::cout << "Payload drop time!" << std::endl;
		//Push back a control move to drop payload.
		GuidanceManeuver gm;
		gm.state = ManeuverState::PayloadDrop;
    gm.done = false;
		RequestGuidanceManeuver(gm);
		coordinateIndex++;
		// If this is the last coordinate of the nav plan, then let's wrap it up here.
		if(coordinateIndex == (int)n.GetNavPlan().coordinates.size()){
			isNavPlanComplete = true;
		}
	}
  // If there's no current maneuver, then we need to calibrate!
  else if((GuidanceManeuverBuffer.empty() || GuidanceManeuverBuffer[GuidanceManeuverIndex].done)
      && !isNavPlanComplete){
    GuidanceManeuver gm;

    // If headings are far apart, we need to turn.
    if(fabs(offAngle) >= 10.0){
      gm.state = ManeuverState::Turn;
      if(offAngle < 0.0){
        gm.turnDirection = 1;
        std::cout << "Turning right!\n";
      }else{
        gm.turnDirection = -1;
        std::cout << "Turning left! " << n.GetHeading() << std::endl;
      }
      gm.speed = 1.0;
      std::cout << "Beginning turn to heading " << m.heading << ".\n";
    }
    //If not, keep it straight.
    else{
      std::cout << "Maintaining course.\n";
      gm.state = ManeuverState::Maintain;
      gm.turnDirection = 0;
      gm.speed = 1.0;
    }
    gm.done = false;
    RequestGuidanceManeuver(gm);
  }

	GuidanceManeuver man = GuidanceManeuverBuffer[GuidanceManeuverIndex];
	switch(man.state){
		case ManeuverState::Calibrate:

			break;
    case ManeuverState::Turn:
      if(fabs(offAngle) < 5.0){
        GuidanceManeuverBuffer[GuidanceManeuverIndex].done = true;
      }
      break;
		case ManeuverState::Maintain:
      if(fabs(offAngle) >= 10.0){
          GuidanceManeuverBuffer[GuidanceManeuverIndex].done = true;
      }
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

  /** @todo Determine how to pass information between Controller and Guider
   * for coordinating payload drops. Control should have the capability to
   * set a flag for when a payload has been dropped appropriately. */
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
