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
Guider::Guider() {
	GuidanceManeuverIndex = -1;
	isNavPlanComplete = false;
	coordinateIndex = 0;
	totalCoordinates = 0;
	savedHeading = 0;
}

int Guider::GetGuidanceManeuverState(){
	if(!GuidanceManeuverBuffer.empty()){ return GetCurrentGuidanceManeuver().state; }
	else{ return -1; }
}

/**
 * This will add a guidance maneuver to the end of the guidance maneuver buffer.
 * @param[in]   gm        - A pre-constructed guidance maneuver
 * @param[out]  gm.index  - Set the index of the guidance maneuver in the buffer.
 */
void Guider::RequestGuidanceManeuver(GuidanceManeuver gm) {
	gm.index = GuidanceManeuverBuffer.size();
	GuidanceManeuverBuffer.push_back(gm);
	return;
}

/**
 * Controls the main operations of the Guider class. This can be separated into three different
 * sections:
 * - **Calculations**: Perform any guidance-specific calculations to be used later in the function->
 * - **Analyze**: Determine based on navigation information if new guidance maneuvers should be added.
 * - **Perform**: Perform actions based on the current guidance maneuver from the buffer.
 * @param[in]   n     - Reference to the navigator.
 * @param[out]  GuidanceManeuverBuffer
 */
void Guider::Run(Navigator* n) {

	double PI = 3.14159265;

	// If value for total coordinates has not been taken from NavPlan yet, update this now.
	if (totalCoordinates != (int)n->GetNavPlan().coordinates.size()) {
		totalCoordinates = (int)n->GetNavPlan().coordinates.size();
	}

	// If coordinateIndex passes through all coordinates, it's assumed that all targets have been acheived.
	// This logic will only be hit on the final pipeline rotation through GNC
	if (coordinateIndex == totalCoordinates) {
		isNavPlanComplete = true;
		TimeModule::Log("GDE", "Guider's Nav Plan comlete! Returning to main for clean-up ops.");
		return;
	}

	// ----------- ANALYZE -------------
	/* This phase of guidance is used to pass new manuevers to the maneuver buffer
	 * upon certain events occuring. The events are also triggered with priority in the
	 * logic that is written below (i.e. Event 1 has higher priority than Event 2). If Event (n)
	 * occurs, then all other events (Event n+1, etc.) will be skipped in this section.
	 *
	 * Event 1: buffer is empty (nothing has been queued yet)
	 *  - Queue the first calibration maneuver.
	 * Event 2: vehicle just completed a payload drop
	 *  - Queue the next calibration manuever to get ready to hit the next target.
	 * Event 3: vehicle in payload drop range for the current drop, and this specific payload drop has not been triggered
	 *  - Queue a payload drop maneuver
	 * Event 4: vehicle encounters an obstacle that it is not currently handling & is not in a payload drop non-move state
	 *  - Queue an avoid-diverge maneuver to avoid this obstacle.
	 * Event 5: previous maneuver is complete and nav-plan is not complete (basically a catch-all)
	 *  - If vehicle is on heading for the next maneuver:
	 *    - Queue a maintain-direction maneuver
	 *  - If vehicle is off course/off heading for next manuever:
	 *    - Queue a vehicle turn maneuver.
	 */

	 // If this is our "first pass", then we need to calibrate.
	if (GuidanceManeuverBuffer.empty())
	{
		GuidanceManeuverIndex++;

		// Request a new calibration guidance maneuver.
		GuidanceManeuver gm;
		gm.state = ManeuverState::Calibrate;
		gm.turnDirection = 0;
		gm.requestedTurnAngle = 0.0;
		gm.currentTurnAngle = 0.0;
		gm.hasBeganDiverging = false;
		gm.maintainTime = 0.0;
		gm.speed = 1.0 * Parser::GetStraightSpeedFactor();
		gm.hasFixedSpeed = false;
		gm.done = false;
		gm.payloadDropComplete = false;
		gm.payloadImageTaken = false;
		gm.accelerationTime = 1.0;
		RequestGuidanceManeuver(gm);
		TimeModule::Log("GDE", "Let's begin! Requesting first calibration.");
		TimeModule::AddMilestone("Speed_" + std::to_string(GetGuidanceManeuverIndex()));
	}
	// if we've just dropped off a payload successfully, then we need to calibrate.
	else if (GuidanceManeuverBuffer[GuidanceManeuverIndex].state == ManeuverState::PayloadDrop &&
		GuidanceManeuverBuffer[GuidanceManeuverIndex].done == true && Parser::GetReCalibrate() == 1)
	{
		GuidanceManeuverIndex++;

		// Request a new calibration guidance maneuver.
		GuidanceManeuver gm;
		gm.state = ManeuverState::Calibrate;
		gm.turnDirection = 0;
		gm.requestedTurnAngle = 0.0;
		gm.currentTurnAngle = 0.0;
		gm.hasBeganDiverging = false;
		gm.maintainTime = 0.0;
		gm.speed = 1.0 * Parser::GetStraightSpeedFactor();
		gm.hasFixedSpeed = false;
		gm.done = false;
		gm.payloadDropComplete = false;
		gm.payloadImageTaken = false;
		gm.accelerationTime = 1.0;
		RequestGuidanceManeuver(gm);
		TimeModule::Log("GDE", "(" + std::to_string(GuidanceManeuverIndex) + "): ReCalibrating after payload drop.");
		TimeModule::AddMilestone("Speed_" + std::to_string(GetGuidanceManeuverIndex()));
	}
	// If we're within payload dropping distance, and if we haven't yet queued a payload drop,
	// then request a new payload drop.
	else if (n->DistanceBetweenCoordinates(n->GetCoordinates(), n->GetNavPlan().coordinates[coordinateIndex]) <= (payloadDropRadius - Parser::GetGPSUncertainty())
		&& !(
			GuidanceManeuverBuffer[GuidanceManeuverIndex].done == false &&
			GuidanceManeuverBuffer[GuidanceManeuverIndex].state == ManeuverState::PayloadDrop
			)
		) {

		// The state that we were currently in is complete now.
		GuidanceManeuverBuffer[GuidanceManeuverIndex].done = true;
		GuidanceManeuverIndex++;

		// Save the heading that was before the payload drop, only if we have accurate data!
		if(GuidanceManeuverBuffer.size() >= 1 && (
					(GuidanceManeuverBuffer[GuidanceManeuverIndex-1].state == ManeuverState::PayloadDrop) ||
					(GuidanceManeuverBuffer[GuidanceManeuverIndex-2].state == ManeuverState::PayloadDrop)
				)
			){
			TimeModule::Log("GDE", "Not saving new heading data, cannot confirm accurate heading data.");
		}else if(GuidanceManeuverBuffer.size() == 0){
			TimeModule::Log("GDE", "THIS SHOULD NOT BE HIT!");
		}else{
			TimeModule::Log("GDE", "Saving our heading for after payload drop.");
			savedHeading = n->GetHeading();
		}

		//Push back a control move to drop payload.
		GuidanceManeuver gm;
		gm.state = ManeuverState::PayloadDrop;
		gm.turnDirection = 0;
		gm.requestedTurnAngle = 0.0;
		gm.currentTurnAngle = 0.0;
		gm.hasBeganDiverging = false;
		gm.maintainTime = 0.0;
		gm.speed = 0.0;
		gm.hasFixedSpeed = false;
		gm.done = false;
		gm.payloadDropComplete = false;
		gm.payloadImageTaken = false;
		gm.accelerationTime	= 2.0;
		RequestGuidanceManeuver(gm);
		TimeModule::Log("GDE", "(" + std::to_string(GuidanceManeuverIndex) + "): Requesting payload drop at maneuver.");
		TimeModule::AddMilestone("Speed_" + std::to_string(GetGuidanceManeuverIndex()));
	}
	// If there's obstructions that we didn't previously know about, then take care of this immediately.
	else if (
		(n->GetPathObstructions().at(0) || n->GetPathObstructions().at(1)) &&
		!GetCurrentGuidanceManeuver().hasBeganDiverging &&
		GuidanceManeuverBuffer[GuidanceManeuverIndex].state != ManeuverState::PayloadDrop) {

		GuidanceManeuverBuffer[GuidanceManeuverIndex].done = true;
		GuidanceManeuverIndex++;
		GuidanceManeuver gm;

		// First, request a turn
		gm.state = ManeuverState::AvoidDiverge;
		gm.done = false;
		gm.speed = -1.0 * Parser::GetBreakFactor();
		gm.turnDirection = 0;
		if (n->GetPathObstructions().at(0)) {
			gm.avoidDirection = -1;
			TimeModule::Log("GDE", "(" + std::to_string(GuidanceManeuverIndex) + "): Requesting obstacle diverge. Sweep to the left - diverge to the right.");
		}
		else {
			gm.avoidDirection = 1;
			TimeModule::Log("GDE", "(" + std::to_string(GuidanceManeuverIndex) + "): Requesting obstacle diverge. Sweep to the right - diverge to the left.");
		}
		gm.payloadDropComplete = false;
		gm.payloadImageTaken = false;
		gm.currentTurnAngle = 0.0;
		gm.requestedTurnAngle = obstacleDivergenceAngle;
		gm.hasBeganDiverging = true;
		gm.avoidDivergeState = 0;
		gm.maintainTime = obstacleDivergenceTime;
		gm.accelerationTime	= 2.0;
		RequestGuidanceManeuver(gm);
		TimeModule::AddMilestone("Speed_" + std::to_string(GetGuidanceManeuverIndex()) + "_" + std::to_string(gm.avoidDivergeState));
	}
	// If we're not within payload dropping range, and have just completed either a calibration,
	// course maintain, or a turn, then see if we need to either turn or maintain course.
	else if (GuidanceManeuverBuffer[GuidanceManeuverIndex].done && !isNavPlanComplete) {

		// If we're just coming from a payload-drop state, we already know our heading.
		double usedHeading = 0.0;
		if (GuidanceManeuverBuffer[GuidanceManeuverIndex].state == ManeuverState::PayloadDrop && Parser::GetReCalibrate() == 0) {
			TimeModule::Log("GDE", "We're using saved heading information!");
			usedHeading = savedHeading;
		}
		else {
			usedHeading = n->GetHeading();
		}
		double x1 = sin(usedHeading * PI / 180.0);
		double y1 = cos(usedHeading * PI / 180.0);

		// Calculate the desired heading to the next coordinate.
		Movement m;
		m = n->CalculateMovement(n->GetCoordinates(), n->GetNavPlan().coordinates[coordinateIndex]);
		double wpLat = n->GetNavPlan().coordinates[coordinateIndex].lat;
		double wpLon = n->GetNavPlan().coordinates[coordinateIndex].lon;
		double x2 = sin(m.heading * PI / 180.0);
		double y2 = cos(m.heading * PI / 180.0);
		double offAngle;
		double dott = x1 * x2 + y1 * y2;      // dot product between [x1, y1] and [x2, y2]
		double det = x1 * y2 - y1 * x2;      // determinant
		offAngle = atan2(det, dott) * 180.0 / PI;  // atan2(y, x) or atan2(sin, cos)
		TimeModule::Log("GDE", "Next waypoint in " + std::to_string(m.distance) + " meters.");

		// If headings are far apart, we need to turn-> CANNOT TURN DIRECTLY AFTER ANOTHER TURn->
		if (fabs(offAngle) >= offAngleDeviate && GetCurrentGuidanceManeuver().state != ManeuverState::Turn) {
			GuidanceManeuverBuffer[GuidanceManeuverIndex].done = true;
			GuidanceManeuverIndex++;
			GuidanceManeuver gm;
			gm.state = ManeuverState::Turn;
			if (offAngle < 0.0) {
				gm.turnDirection = 1;
				TimeModule::Log("GDE", "(" + std::to_string(GuidanceManeuverIndex) + "): Turning right " + std::to_string(fabs(offAngle)) + " degrees.");
			}
			else {
				gm.turnDirection = -1;
				TimeModule::Log("GDE", "(" + std::to_string(GuidanceManeuverIndex) + "): Turning left " + std::to_string(fabs(offAngle)) + " degrees.");
			}
			gm.requestedTurnAngle = fabs(offAngle);
			gm.currentTurnAngle = 0.0;
			gm.hasBeganDiverging = false;
			gm.maintainTime = 0.0;
			gm.speed = 1.0 * Parser::GetTurnSpeedFactor();
			gm.hasFixedSpeed = false;
			gm.done = false;
			gm.payloadDropComplete = false;
			gm.payloadImageTaken = false;

			// If we're already at the correct speed, then note this!
			if(GuidanceManeuverBuffer[GuidanceManeuverIndex-1].speed == gm.speed){
				gm.accelerationTime	= 0.0;
			}else{
				gm.accelerationTime	= 1.5;
			}

			RequestGuidanceManeuver(gm);
			TimeModule::AddMilestone("Speed_" + std::to_string(GetGuidanceManeuverIndex()));
		}
		// If our heading is relatively nominal, we will maintain our course.
		else {
			GuidanceManeuverBuffer[GuidanceManeuverIndex].done = true;
			GuidanceManeuverIndex++;
			GuidanceManeuver gm;

			gm.state = ManeuverState::Maintain;
			gm.turnDirection = 0;
			gm.requestedTurnAngle = 0.0;
			gm.currentTurnAngle = 0.0;
			gm.hasBeganDiverging = false;
			gm.maintainTime = minimumMaintainTime;
			gm.speed = 1.0 * Parser::GetStraightSpeedFactor();
			gm.hasFixedSpeed = false;
			gm.done = false;
			gm.payloadDropComplete = false;
			gm.payloadImageTaken = false;

			// If we're already at the correct speed, then note this!
			if(GuidanceManeuverBuffer[GuidanceManeuverIndex-1].speed == gm.speed){
				gm.accelerationTime	= 0.0;
			}else{
				gm.accelerationTime	= 1.5;
			}

			RequestGuidanceManeuver(gm);
			TimeModule::Log("GDE", "(" + std::to_string(GuidanceManeuverIndex) + "): Requesting course maintain.");
			TimeModule::AddMilestone("Speed_" + std::to_string(GetGuidanceManeuverIndex()));
		}
	}

	//std::cout << "Maneuver: " << GuidanceManeuverIndex << "\n";


	/* ----------- CHECK --------------
	 * This logic is in charge of making decisions about the current state of
	 * the current guidance maneuver in the buffer. This logic will set the "done" flag for the current
	 * guidance maneuver if the logic determines that the guidance maneuver has been completed. The
	 * logic that is triggered below depends on which type of maneuver is currently being performed.
	 * Many of the checks below are only checked once the vehicle has acheived a fixed speed after
	 * acceleration or deceleration. The hasFixedSpeed flax is set in the Controller after accelerating.
	 *
	 * Calibrate:
	 *  - If vehicle has calibrated for requested calibration time:
	 *    - DONE
	 *    - If vehicle set to optimize nav-plan, then re-optimize now.
	 * Turn:
	 *  - Update current turn angle using integrator
	 *  - If vehicle has turned the total requested turn amount:
	 *    - DONE
	 * Maintain:
	 *  - If vehicle has maintained course for total requested amount of time:
	 *    - DONE
	 * AvoidDiverge:
	 *  - If vehicle has completed the reverse avoidance turn:
	 *    - If vehicle has maintained path for the requested obstacle avoidance time:
	 *      - DONE
	 *  - If vehicle has not completed turning yet:
	 *    - Update current turn angle using integrator
	 *    - If vehicle has completed turn:
	 *      - Set completed turn flag.
	 */
	GuidanceManeuver* man = &GuidanceManeuverBuffer[GuidanceManeuverIndex];
	switch (man->state) {
	case ManeuverState::Calibrate:

		if (!man->hasFixedSpeed)
			break;

		// If the total required calibration time has elapsed, then the maneuver is complete.
		if (TimeModule::GetElapsedTime("Calibration_" + std::to_string(GuidanceManeuverIndex)) >= calibrationTime) {
			GuidanceManeuverBuffer[GuidanceManeuverIndex].done = true;
			TimeModule::Log("GDE", "Calibration maneuver complete.");

			// Reoptimize if
			if (Parser::GetReOptimize() == 1 && (
						(Parser::GetReCalibrate() == 1) ||
						(GuidanceManeuverBuffer.size() == 1)
					)
				){
				TimeModule::Log("GDE", "Re-Optimizing Nav-Plan after calibration maneuver.");
				n->ConstructNavPlan(coordinateIndex);
				int step = 0;
				for (int i = coordinateIndex; i < n->GetNavPlan().movements.size()-1; i++){
					TimeModule::Log("NAV", "Step " + std::to_string(step+1) + ": move " + std::to_string(n->GetNavPlan().movements[i].distance)
						+ " meters at heading " + std::to_string(n->GetNavPlan().movements[i].heading)
						+ " to payload point #" + std::to_string(i+1));
						step++;
				}
			}else{
				TimeModule::Log("GDE", "No need to re-optimize after this calibration maneuver.");
			}

		}
		break;
	case ManeuverState::Turn:
	{

		if (!man->hasFixedSpeed)
			break;

		// Use a basic integrator to estimate the amount of time we have been turning.
		double dtTurn = TimeModule::GetElapsedTime("Turn_" + std::to_string(GuidanceManeuverIndex));
		man->currentTurnAngle = turnFactorDPS * dtTurn;

		// If the total off-angle has subsided, then we can consider the turn to be complete.
		if (man->currentTurnAngle >= man->requestedTurnAngle) {
			GuidanceManeuverBuffer[GuidanceManeuverIndex].done = true;
			TimeModule::Log("GDE", "Turn maneuver complete.");
		}
		break;
	}
	case ManeuverState::Maintain:

		if (!man->hasFixedSpeed)
			break;

		// If the off-angle grows too large, then our maintenance maneuver is over, and a turn maneuver will be added next pass.
		if (TimeModule::GetElapsedTime("Maintain_" + std::to_string(GuidanceManeuverIndex)) >= minimumMaintainTime) {
			GuidanceManeuverBuffer[GuidanceManeuverIndex].done = true;
			TimeModule::Log("GDE", "Course maintain maneuver complete.");
		}
		break;

	case ManeuverState::AvoidDiverge:
	{

		if(man->avoidDivergeState == 3)
		{
			man->hasBeganDiverging = false;

			if (!man->hasFixedSpeed)
				break;
			if (TimeModule::GetElapsedTime("Diverge_" + std::to_string(GuidanceManeuverIndex)) >= man->maintainTime) {
				GuidanceManeuverBuffer[GuidanceManeuverIndex].done = true;
				TimeModule::Log("GDE", "Avoid-Diverge maneuver complete.");
			}

		}
		else if(man->avoidDivergeState == 2)
		{

			if(!man->hasFixedSpeed)
				break;
			man->hasFixedSpeed = false;
			man->speed = 1.0 * Parser::GetStraightSpeedFactor();
			man->turnDirection = 0;
			man->avoidDivergeState = 3;
			man->accelerationTime = 2.0;
			TimeModule::Log("GDE", "Received stopping signal again. Time to move forward.");
			TimeModule::AddMilestone("Speed_" + std::to_string(GetGuidanceManeuverIndex()) + "_" + std::to_string(man->avoidDivergeState));

		}
		else if(man->avoidDivergeState == 1)
		{

			if(!man->hasFixedSpeed)
				break;

			double dtTurn = TimeModule::GetElapsedTime("Avoid_" + std::to_string(GuidanceManeuverIndex));
			man->currentTurnAngle = turnFactorDPS * dtTurn;
			if (man->currentTurnAngle >= man->requestedTurnAngle) {
				man->hasFixedSpeed = false;
				man->speed = 0.0;
				man->turnDirection = 0;
				man->avoidDivergeState = 2;
				man->accelerationTime = 1.0;
				TimeModule::Log("GDE", "Turn has been complete. Stopping again.");
				TimeModule::AddMilestone("Speed_" + std::to_string(GetGuidanceManeuverIndex()) + "_" + std::to_string(man->avoidDivergeState));
			}

		}
		else
		{

			if(!man->hasFixedSpeed)
				break;
			man->hasFixedSpeed = false;
			man->speed = -1.5 * Parser::GetTurnSpeedFactor();
			man->turnDirection = man->avoidDirection;
			man->avoidDivergeState = 1;
			man->accelerationTime = 1.0;
			TimeModule::Log("GDE", "Received stopping signal. Time to back up.");
			TimeModule::AddMilestone("Speed_" + std::to_string(GetGuidanceManeuverIndex()) + "_" + std::to_string(man->avoidDivergeState));

		}

		break;
	}
	case ManeuverState::PayloadDrop:

		if (!man->hasFixedSpeed)
			break;

		// If the payload-drop complete flag has been triggered, then this guidance maneuver has been completed.
		if (man->payloadDropComplete && man->payloadImageTaken) {
			GuidanceManeuverBuffer[GuidanceManeuverIndex].done = true;
			TimeModule::Log("GDE", "CTL sent payload-drop and image-taken signals.");
			coordinateIndex++;

			if (coordinateIndex != n->GetNavPlan().coordinates.size() && Parser::GetReOptimize() && !Parser::GetReCalibrate()) {
				TimeModule::Log("GDE", "Re-Optimizing Nav-Plan after payload drop.");
				n->ConstructNavPlan(coordinateIndex);
				int step = 0;
				for (int i = coordinateIndex; i < n->GetNavPlan().movements.size()-1; i++){
					TimeModule::Log("NAV", "Step " + std::to_string(step+1) + ": move " + std::to_string(n->GetNavPlan().movements[i].distance)
						+ " meters at heading " + std::to_string(n->GetNavPlan().movements[i].heading)
						+ " to payload point #" + std::to_string(i+1));
						step++;
				}
			}

		}
		break;
	}

	return;
}

std::vector<GuidanceManeuver> Guider::GetGuidanceManeuverBuffer() { return GuidanceManeuverBuffer; }
GuidanceManeuver& Guider::GetCurrentGuidanceManeuver() { return GuidanceManeuverBuffer[GuidanceManeuverIndex]; }
int Guider::GetGuidanceManeuverIndex() { return GuidanceManeuverIndex; }
bool Guider::IsNavPlanComplete() { return isNavPlanComplete; }
