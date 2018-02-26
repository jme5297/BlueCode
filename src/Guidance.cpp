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

	// If coordinateIndex passes through all coordinates, it's assumed that all targets have been acheived.
	if (coordinateIndex == (int)n->GetNavPlan().coordinates.size()) {
		isNavPlanComplete = true;
		std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "]: Guider's Nav Plan comlete! Returning to main for clean-up ops.\n";
		return;
	}
	// ----------- ANALYZE -------------
	// If this is our "first pass", then we need to calibrate.
	if (GuidanceManeuverBuffer.empty())
	{
		GuidanceManeuverIndex++;

		// Request a new calibration guidance maneuver.
		GuidanceManeuver gm;
		gm.state = ManeuverState::Calibrate;
		gm.speed = 1.0;
		gm.turnDirection = 0;
		gm.done = false;
		gm.speedRate = Parser::GetAccFactor() * Parser::GetRefresh_GUID();
		RequestGuidanceManeuver(gm);
		std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "]: Calibrating...\n";
	}
	// if we've just dropped off a payload successfully, then we need to calibrate.
	else if (GuidanceManeuverBuffer[GuidanceManeuverIndex].state == ManeuverState::PayloadDrop &&
		GuidanceManeuverBuffer[GuidanceManeuverIndex].done == true)
	{
		GuidanceManeuverIndex++;

		// Request a new calibration guidance maneuver.
		GuidanceManeuver gm;
		gm.state = ManeuverState::Calibrate;
		gm.speed = 1.0;
		gm.turnDirection = 0;
		gm.done = false;
		gm.speedRate = Parser::GetAccFactor() * Parser::GetRefresh_GUID();
		RequestGuidanceManeuver(gm);
		std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "]: Calibrating...\n";
	}
	// If we're within payload dropping distance, and if we haven't yet queued a payload drop,
	// then request a new payload drop.
	else if (n->DistanceBetweenCoordinates(n->GetCoordinates(), n->GetNavPlan().coordinates[coordinateIndex]) <= (payloadDropRadius - Parser::GetGPSUncertainty())
		&& !(
			GuidanceManeuverBuffer[GuidanceManeuverIndex].done == false &&
			GuidanceManeuverBuffer[GuidanceManeuverIndex].state == ManeuverState::PayloadDrop
			)
		) {
		std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "]: Payload drop time!" << std::endl;

		// The state that we were currently in is complete now.
		GuidanceManeuverBuffer[GuidanceManeuverIndex].done = true;
		GuidanceManeuverIndex++;

		//Push back a control move to drop payload.
		GuidanceManeuver gm;
		gm.state = ManeuverState::PayloadDrop;
		gm.done = false;
		gm.speed = 0.0;
		gm.payloadDropComplete = false;
		gm.payloadImageTaken = false;
		gm.speedRate = Parser::GetAccFactor() * Parser::GetRefresh_GUID();
		RequestGuidanceManeuver(gm);
	}
	// If there's obstructions that we didn't previously know about, then take care of this immediately.
	else if ((n->GetPathObstructions().at(0) || n->GetPathObstructions().at(1)) && !GetCurrentGuidanceManeuver().hasBeganDiverging) {
		GuidanceManeuverBuffer[GuidanceManeuverIndex].done = true;

		// If this was the last coordinate, then lets wrap it up here.
		/*
		if (coordinateIndex == (int)n->GetNavPlan().coordinates.size()-1) {
			isNavPlanComplete = true;
			std::cout << "==== Returned to original location. We're done here. ====\n";
			std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "]: Guider's Nav Plan comlete! Returning to main for clean-up ops.\n";
			return;
		}
		*/

		GuidanceManeuverIndex++;
		GuidanceManeuver gm;

		// First, request a turn->
		gm.state = ManeuverState::AvoidDiverge;
		if (n->GetPathObstructions().at(0)) {
			gm.turnDirection = -1;
		}
		else {
			gm.turnDirection = 1;
		}
		gm.currentTurnAngle = 0.0;
		gm.requestedTurnAngle = obstacleDivergenceAngle;
		gm.hasBeganDiverging = false;
		gm.speed = -0.25;
		gm.speedRate = Parser::GetAccFactorObs() * Parser::GetRefresh_GUID();
		gm.maintainTime = obstacleDivergenceTime;
		gm.done = false;
		RequestGuidanceManeuver(gm);

		std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "]: Diverging obstacle!\n";
	}
	// If we're not within payload dropping range, and have just completed either a calibration,
	// course maintain, or a turn, then see if we need to either turn or maintain course.
	else if (GuidanceManeuverBuffer[GuidanceManeuverIndex].done && !isNavPlanComplete) {

		std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "]: -------------- Grabbing current GPS information.\n";
		Movement m;
		m = n->CalculateMovement(n->GetCoordinates(), n->GetNavPlan().coordinates[coordinateIndex]);
		double x1 = sin(n->GetHeading() * PI / 180.0);
		double y1 = cos(n->GetHeading() * PI / 180.0);
		double x2 = sin(m.heading * PI / 180.0);
		double y2 = cos(m.heading * PI / 180.0);
		double offAngle;
		double dott = x1 * x2 + y1 * y2;      // dot product between [x1, y1] and [x2, y2]
		double det = x1 * y2 - y1 * x2;      // determinant
		offAngle = atan2(det, dott) * 180.0 / PI;  // atan2(y, x) or atan2(sin, cos)

		// If headings are far apart, we need to turn-> CANNOT TURN DIRECTLY AFTER ANOTHER TURn->
		if (fabs(offAngle) >= offAngleDeviate && GetCurrentGuidanceManeuver().state != ManeuverState::Turn) {
			GuidanceManeuverBuffer[GuidanceManeuverIndex].done = true;
			GuidanceManeuverIndex++;
			GuidanceManeuver gm;
			gm.state = ManeuverState::Turn;
			if (offAngle < 0.0) {
				gm.turnDirection = 1;
				std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "]: Turning right " << std::to_string(fabs(offAngle)) << " degrees.\n";
			}
			else {
				gm.turnDirection = -1;
				std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "]: Turning left " << std::to_string(fabs(offAngle)) << " degrees.\n";
			}
			gm.currentTurnAngle = 0.0;
			gm.requestedTurnAngle = fabs(offAngle);
			gm.speed = 0.25;
			gm.speedRate = Parser::GetAccFactor() * Parser::GetRefresh_GUID();
			//TimeModule::AddMilestone("Turn_" + std::to_string(GuidanceManeuverIndex));
			gm.done = false;
			RequestGuidanceManeuver(gm);
		}
		// If our heading is relatively nominal, we will maintain our course.
		else {
			GuidanceManeuverBuffer[GuidanceManeuverIndex].done = true;
			GuidanceManeuverIndex++;
			GuidanceManeuver gm;
			double dist = n->DistanceBetweenCoordinates(n->GetCoordinates(), n->GetNavPlan().coordinates[coordinateIndex]);
			gm.state = ManeuverState::Maintain;
			gm.turnDirection = 0;
			gm.speed = 1.0;
			gm.speedRate = Parser::GetAccFactor() * Parser::GetRefresh_GUID();
			// We will maintain course, but never undershoot the minimum calibration time for maintaing course.
			double time = dist / maxVehicleSpeed * 0.5;
			gm.maintainTime = (time < minimumMaintainTime) ? minimumMaintainTime : time;
			//TimeModule::AddMilestone("Maintain_" + std::to_string(GuidanceManeuverIndex));
			gm.done = false;
			RequestGuidanceManeuver(gm);
			std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "]: Maintaining course for " << std::to_string(gm.maintainTime) << " seconds.\n";
		}
	}


	/* ----------- PERFORM --------------
	 * This logic is in charge of making decisions about the current state of
	 * the current guidance maneuver in the buffer.
	 */
	GuidanceManeuver* man = &GuidanceManeuverBuffer[GuidanceManeuverIndex];

	switch (man->state) {
	case ManeuverState::Calibrate:

		if(!man->hasFixedSpeed)
			break;

		// If the total required calibration time has elapsed, then the maneuver is complete.
		if (TimeModule::GetElapsedTime("Calibration_" + std::to_string(GuidanceManeuverIndex)) >= calibrationTime) {
			GuidanceManeuverBuffer[GuidanceManeuverIndex].done = true;
			std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "]: Calibration complete.\n";

			if(Parser::GetOptimize()){
				n->ConstructNavPlan(coordinateIndex);
			}
		}
		break;
	case ManeuverState::Turn:
	{

		if(!man->hasFixedSpeed)
			break;

		// If the total off-angle has subsided, then we can consider the turn to be complete.
		double dtTurn = TimeModule::GetElapsedTime("Turn_" + std::to_string(GuidanceManeuverIndex));
		man->currentTurnAngle = turnFactorDPS * dtTurn;
		if (man->currentTurnAngle >= man->requestedTurnAngle) {
			GuidanceManeuverBuffer[GuidanceManeuverIndex].done = true;
			std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "]: Turn complete.\n";
		}
		break;
	}
	case ManeuverState::Maintain:

		if(!man->hasFixedSpeed)
			break;

		// If the off-angle grows too large, then our maintenance maneuver is over, and a turn maneuver will be added next pass.
		if (TimeModule::GetElapsedTime("Maintain_" + std::to_string(GuidanceManeuverIndex)) >= man->maintainTime) {
			GuidanceManeuverBuffer[GuidanceManeuverIndex].done = true;
			std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "]: Done maintaining course.\n";
		}
		break;

	case ManeuverState::AvoidDiverge:
	{
		if (man->currentTurnAngle >= man->requestedTurnAngle) {
			man->hasBeganDiverging = false;
			man->speed = 1.0;
			man->turnDirection = 0;
			man->speedRate = 0.5*Parser::GetRefresh_GUID();
			if (TimeModule::GetElapsedTime("Avoid_" + std::to_string(GuidanceManeuverIndex)) >= man->maintainTime) {
				man->done = true;
				std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "]: Done avoiding obstacle.\n";
			}
		}else{
			double dtTurn = TimeModule::GetElapsedTime("Avoid_" + std::to_string(GuidanceManeuverIndex));
			man->currentTurnAngle = turnFactorDPS * dtTurn;
			man->hasBeganDiverging = true;
			if (man->currentTurnAngle >= man->requestedTurnAngle) {
				man->hasFixedSpeed = false;
				man->hasBeganDiverging = false;
				man->speed = 1.0;
				man->turnDirection = 0;
				man->speedRate = 0.5*Parser::GetRefresh_GUID();
			}
		}
		break;
	}
	case ManeuverState::AvoidConverge:

		break;

	case ManeuverState::PayloadDrop:

		if(!man->hasFixedSpeed)
			break;

		// If the payload-drop complete flag has been triggered, then this guidance maneuver has been completed.
		if (man->payloadDropComplete && man->payloadImageTaken) {
			GuidanceManeuverBuffer[GuidanceManeuverIndex].done = true;
			std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "]: Payload dropped, image taken." << std::endl;
			coordinateIndex++;
		}
		break;
	}

	return;
}

std::vector<GuidanceManeuver> Guider::GetGuidanceManeuverBuffer() { return GuidanceManeuverBuffer; }
GuidanceManeuver& Guider::GetCurrentGuidanceManeuver() { return GuidanceManeuverBuffer[GuidanceManeuverIndex]; }
int Guider::GetGuidanceManeuverIndex() { return GuidanceManeuverIndex; }
bool Guider::IsNavPlanComplete() { return isNavPlanComplete; }
