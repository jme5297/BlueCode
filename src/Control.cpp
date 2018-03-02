#include <Control.h>

#ifdef TEST_PWM
#include <thread>
#include <prussdrv.h>
#include <pruss_intc_mapping.h>
#define PRU_NUM   0
#endif

#ifdef SIM
using namespace Plant;
#endif

using namespace Guidance;
using namespace Control;
using namespace sensors;

// Static variable for current wheel speed (in double form)
double currentWheelSpeed;

// Debugging variables
int motorCount;
int runCount;

void ControlMotors()
{
	unsigned int curWheelSpeedI = 1;
	// Initialize structure used by prussdrv_pruintc_intc
	tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;

	while(true){
		curWheelSpeedI = (int)(currentWheelSpeed*100.0);
		curWheelSpeedI = (curWheelSpeedI == 0) ? 1 : curWheelSpeedI;

		prussdrv_init ();
		prussdrv_open (PRU_EVTOUT_0);
		// Map PRU intrrupts
		prussdrv_pruintc_init(&pruss_intc_initdata);
		// write duty cycle on PRU memory
		prussdrv_pru_write_memory(PRUSS0_PRU0_DATARAM, 0, &init_duty_cycle, 4);

		unsigned int sampletimestep = 10;  //delay factor (10 default, 624 for 1600 Hz)
		// write it into the next word location in memory (i.e. 4-bytes later)
		prussdrv_pru_write_memory(PRUSS0_PRU0_DATARAM, 1, &sampletimestep, 4);
		// Load and execute binary on PRU
		prussdrv_exec_program (PRU_NUM, "./pwm_test.bin");
		// Wait for event completion from PRU
		int n = prussdrv_pru_wait_event (PRU_EVTOUT_0);
	}
}

/// @todo This should not be hard-coded if a generalized model is desired.
Controller::Controller() {
	currentVehicleMode = Parser::GetControlMode();
	currentWheelSpeed = 0.0;
}

void Controller::InitializeMotorControl(){
	#ifdef TEST_PWM
	std::cout << "Creating a thread for motor control...\n";
	std::thread runMotors(ControlMotors);
	runMotors.detach();
	#endif
}

/// @todo Determine if all of these switches are necessary.
void Controller::Run(Guider* g, SensorHub* sh) {

	runCount++;
	motorCount = 0;

	// Update the speeds.
	if(!g->GetCurrentGuidanceManeuver().hasFixedSpeed){
		if(!(fabs(currentWheelSpeed - g->GetCurrentGuidanceManeuver().speed) <= 2.0 * 0.005)){
			double sign = (g->GetCurrentGuidanceManeuver().speed - currentWheelSpeed) / fabs(currentWheelSpeed - g->GetCurrentGuidanceManeuver().speed);
			currentWheelSpeed += sign*g->GetCurrentGuidanceManeuver().speedRate;
			SetWheelSpeed(currentWheelSpeed);
			SetWheelSteering(0.0);
		}else{
			g->GetCurrentGuidanceManeuver().hasFixedSpeed = true;
			switch (g->GetCurrentGuidanceManeuver().state) {
				case ManeuverState::Calibrate:
					TimeModule::AddMilestone("Calibration_" + std::to_string(g->GetGuidanceManeuverIndex()));
					break;
				case ManeuverState::Turn:
					TimeModule::AddMilestone("Turn_" + std::to_string(g->GetGuidanceManeuverIndex()));
				case ManeuverState::Maintain:
					TimeModule::AddMilestone("Maintain_" + std::to_string(g->GetGuidanceManeuverIndex()));
					break;
				case ManeuverState::AvoidDiverge:
					TimeModule::AddMilestone("Avoid_" + std::to_string(g->GetGuidanceManeuverIndex()));
					break;
				case ManeuverState::AvoidConverge:
					break;
				case ManeuverState::PayloadDrop:
					TimeModule::AddMilestone("PayloadDrop_" + std::to_string(g->GetGuidanceManeuverIndex()));
					break;
			}
		}
		return;
	}

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

		// If we're on our last leg (return-to-home), then no need to drop payload.
		if(g->coordinateIndex == g->totalCoordinates-1){
			g->GetCurrentGuidanceManeuver().payloadDropComplete = true;
			g->GetCurrentGuidanceManeuver().payloadImageTaken = true;
			std::cout << "=== RETURNED TO HOME ===\n";
			return;
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
