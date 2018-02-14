#include <Control.h>

#ifdef SIM
using namespace Plant;
#endif

using namespace Guidance;
using namespace Control;
using namespace sensors;

/// @todo This should not be hard-coded if a generalized model is desired.
Controller::Controller() {
	currentVehicleMode = Parser::GetControlMode();
}

/// @todo Determine if all of these switches are necessary.
void Controller::Run(Guider* g, SensorHub* sh) {

	// If the NAV plan is complete, then stop the vehicle and return->
	if (g->IsNavPlanComplete()) {
		std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "]: Nav Plan complete. Stopping vehicle.\n";
		switch (currentVehicleMode) {
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

	// If the buffer is empty, then don't run anything.
	if (g->GetGuidanceManeuverBuffer().empty() || g->GetCurrentGuidanceManeuver().done) {
		switch (currentVehicleMode) {
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

	// If the current maneuver is a paylod drop, run payload drop functions.
	if (g->GetCurrentGuidanceManeuver().state == ManeuverState::PayloadDrop) {
		switch (currentVehicleMode) {
		case VehicleMode::Wheel:
			SetWheelSpeed(0.0);
			SetWheelSteering(0.0);
			break;
		case VehicleMode::Track:
			SetMotorSpeeds(0.0);
			break;
		}
		PayloadDrop(g, sh);
		return;
	}

	// Lastly, perform normal operations if none of the above were triggered.
	switch (currentVehicleMode) {
	case VehicleMode::Wheel:
		SetWheelSpeed(g->GetCurrentGuidanceManeuver().speed);
		SetWheelSteering((double)g->GetCurrentGuidanceManeuver().turnDirection * maxTurnSteering);
		break;
	case VehicleMode::Track:
		// Note, this functionality is not built yet.
		SetMotorSpeeds(g->GetCurrentGuidanceManeuver().speed);
		break;
	}
	return;

	return;
}

/**
 * \note PayloadDrop also handles camera image operations.
 */
void Controller::PayloadDrop(Guider* g, SensorHub* sh) {

	if (!g->GetCurrentGuidanceManeuver().payloadDropComplete) {
		if (TimeModule::GetElapsedTime("PayloadDrop_" + std::to_string(g->GetGuidanceManeuverIndex())) >= g->GetPayloadServoTime()) {
			g->GetCurrentGuidanceManeuver().payloadDropComplete = true;
			std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "]: Payload deployed!\n";
			// payloadServo = 0.0
		}
		else {
			// Functionality goes here for controlling the payload drop mechanism.
			// payloadServo = 1.0

			return;
		}
	}

	// Attempt to take an image after the payload has been dropped.
	bool imageTaken = sh->GetCamera()->TakeImage(g->GetCurrentGuidanceManeuver().index);
	if (imageTaken) {
		g->GetCurrentGuidanceManeuver().payloadImageTaken = true;
		std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "]: Controller received successful image signal!\n";
	}
	else {
		std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "]: Controller says image taking has failed.\n";
	}
	return;
}

void Controller::SetWheelSpeed(double s) {
	wheelSpeedN = s;

#ifdef SIM
	PlantModel::GetVehicle()->wheelSpeedN = s;
#else
	// Actual speed control goes here
#endif
}


void Controller::SetWheelSteering(double s) {
	wheelSteeringN = s;

#ifdef SIM
	PlantModel::GetVehicle()->wheelSteeringN = s;
#else
	// Actual steering control goes here
#endif
}

void Controller::SetMotorSpeeds(double speed) {

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

void Controller::SetMotorLSpeed(double speed) {

	motorLSpeed = speed;

#ifdef SIM
	PlantModel::GetVehicle()->motL.val = speed;
#else
	// Actual motor control goes here
#endif

	return;
}

void Controller::SetMotorRSpeed(double speed) {

	motorRSpeed = speed;

#ifdef SIM
	PlantModel::GetVehicle()->motR.val = speed;
#else
	// Actual motor control goes here
#endif

	return;
}

void Controller::SetCurrentVehicleMode(VehicleMode vm) { currentVehicleMode = vm; }
VehicleMode Controller::GetCurrentVehicleMode() { return currentVehicleMode; }
double Controller::GetMotorLSpeed() { return motorLSpeed; }
double Controller::GetMotorRSpeed() { return motorRSpeed; }
double Controller::GetWheelSpeed() { return wheelSpeedN; }
double Controller::GetWheelSteering() { return wheelSteeringN; }
void Controller::SetMaxTurnSteering(double d) { maxTurnSteering = d; }
// void Controller::SetMaxCameraAttempts(Parser::GetMaxCameraAttempts());