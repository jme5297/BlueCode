#include <Guidance.h>

using namespace Times;
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
	gm.index = GuidanceManeuverBuffer.size();
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

  double PI = 3.14159265;

	// If coordinateIndex passes through all coordinates, it's assumed that all targets have been acheived.
	if (coordinateIndex == (int)n.GetNavPlan().coordinates.size()) {
		isNavPlanComplete = true;
		std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "]: Guider's Nav Plan comlete! Returning to main for clean-up ops.\n";
		return;
	}

  /* -------- CALCULATIONS ------------
   * Calculate the angle in-between our heading and the desired heading of our next target.
   * A positive angle is counter-clockwise from the vehicle's heading.
   * A negative angle is clockwise from the vehicle's heading.
   */
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

  // ----------- ANALYZE -------------
  // If this is our "first pass", or if we've just dropped off a payload successfully, then we need to calibrate.
  if (GuidanceManeuverBuffer.empty() ||
      (
        GuidanceManeuverBuffer[GuidanceManeuverIndex].state == ManeuverState::PayloadDrop &&
        GuidanceManeuverBuffer[GuidanceManeuverIndex].done == true
      )
    ){

    GuidanceManeuverIndex++;

    // Request a new calibration guidance maneuver.
    GuidanceManeuver gm;
    gm.state = ManeuverState::Calibrate;
    gm.speed = 1.0;
    gm.turnDirection = 0;
    gm.calibrationTime = 0.0;
    gm.done = false;
    RequestGuidanceManeuver(gm);
    TimeModule::AddMilestone("Calibration_" + std::to_string(GuidanceManeuverIndex));
    std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "]: Calibrating...\n";
  }
  // If we're within payload dropping distance, and if we haven't yet queued a payload drop,
  // then request a new payload drop.
	else if(n.DistanceBetweenCoordinates(n.GetCoordinates(), n.GetNavPlan().coordinates[coordinateIndex]) <= payloadDistance
    &&  !(
        GuidanceManeuverBuffer[GuidanceManeuverIndex].done == false &&
        GuidanceManeuverBuffer[GuidanceManeuverIndex].state == ManeuverState::PayloadDrop
        )
    ){
    std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "]: Payload drop time!" << std::endl;

    // The state that we were currently in is complete now.
    GuidanceManeuverBuffer[GuidanceManeuverIndex].done = true;
    GuidanceManeuverIndex++;

		//Push back a control move to drop payload.
		GuidanceManeuver gm;
		gm.state = ManeuverState::PayloadDrop;
    gm.done = false;
    gm.payloadDropComplete = false;
    gm.payloadServoTime = 2.0;
    gm.payloadImageTaken = false;
		RequestGuidanceManeuver(gm);

    // Add a reference to this time.
    TimeModule::AddMilestone("PayloadDrop_" + std::to_string(GuidanceManeuverIndex));
	}
  /// @todo This is where functionality would be added for obstacle avoidance.
  // If we're not within payload dropping range, and have just completed either a calibration,
  // course maintain, or a turn, then see if we need to either turn or maintain course.
  else if(GuidanceManeuverBuffer[GuidanceManeuverIndex].done
      && (
          GuidanceManeuverBuffer[GuidanceManeuverIndex].state == ManeuverState::Maintain ||
          GuidanceManeuverBuffer[GuidanceManeuverIndex].state == ManeuverState::Calibrate ||
          GuidanceManeuverBuffer[GuidanceManeuverIndex].state == ManeuverState::Turn
        )
      && !isNavPlanComplete){

    GuidanceManeuverIndex++;

    GuidanceManeuver gm;

    // If headings are far apart, we need to turn.
    /// @todo These angle values should not be hard-coded.
    if(fabs(offAngle) >= 10.0){
      gm.state = ManeuverState::Turn;
      if(offAngle < 0.0){
        gm.turnDirection = 1;
        std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "]: Turning right!\n";
      }else{
        gm.turnDirection = -1;
        std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "]: Turning left!\n";
      }
      gm.speed = 1.0;
    }
    // If our heading is relatively nominal, we will maintain our course.
    else{
      std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "]: Maintaining course.\n";
      gm.state = ManeuverState::Maintain;
      gm.turnDirection = 0;
      gm.speed = 1.0;
    }
    gm.done = false;
    RequestGuidanceManeuver(gm);
  }

  /* ----------- PERFORM --------------
   * This logic is in charge of making decisions about the current state of
   * the current guidance maneuver in the buffer.
   */
	GuidanceManeuver man = GuidanceManeuverBuffer[GuidanceManeuverIndex];
	switch(man.state){

		case ManeuverState::Calibrate:
      // If the total required calibration time has elapsed, then the maneuver is complete.
      if(TimeModule::GetElapsedTime("Calibration_" + std::to_string(GuidanceManeuverIndex)) >= man.calibrationTime){
        GuidanceManeuverBuffer[GuidanceManeuverIndex].done = true;
        std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "]: Calibration complete.\n";
      }
			break;

    case ManeuverState::Turn:
      // If the total off-angle has subsided, then we can consider the turn to be complete.
      /// @todo This value should not be hard-coded.
      if(fabs(offAngle) < 5.0){
        GuidanceManeuverBuffer[GuidanceManeuverIndex].done = true;
        std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "]: Turn complete.\n";
      }
      break;

		case ManeuverState::Maintain:
      // If the off-angle grows too large, then our maintenance maneuver is over, and a turn maneuver will be added next pass.
      /// @todo This value should not be hard-coded.
      if(fabs(offAngle) >= 10.0){
          GuidanceManeuverBuffer[GuidanceManeuverIndex].done = true;
          std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "]: We are deviating!\n";
      }
			break;

		case ManeuverState::AvoidDiverge:

			break;

		case ManeuverState::AvoidConverge:

			break;

		case ManeuverState::PayloadDrop:
      // If the payload-drop complete flag has been triggered, then this guidance maneuver has been completed.
      if(man.payloadDropComplete && man.payloadImageTaken){
        GuidanceManeuverBuffer[GuidanceManeuverIndex].done = true;
        std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "]: Payload dropped, image taken." << std::endl;
        // Add the coordinate index counter.
        coordinateIndex++;
      }
			break;
	}

  return;
}

std::vector<GuidanceManeuver> Guider::GetGuidanceManeuverBuffer(){ return GuidanceManeuverBuffer; }
GuidanceManeuver& Guider::GetCurrentGuidanceManeuver(){ return GuidanceManeuverBuffer[GuidanceManeuverIndex]; }
int Guider::GetGuidanceManeuverIndex(){ return GuidanceManeuverIndex; }
bool Guider::IsNavPlanComplete() { return isNavPlanComplete; }
void Guider::SetPayloadDistance(double d) { payloadDistance = d; }
double Guider::GetPayloadDistance() { return payloadDistance; }
