#include <Control.h>

#ifdef TEST_PWM
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
double currentWheelSteering;
double currentPayloadServo;

// Debugging variables
int motorCount;
int runCount;

void ControlMotors()
{
	#ifdef TEST_PWM
	unsigned int curWheelSpeedI = 1;
	// Initialize structure used by prussdrv_pruintc_intc
	tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;

	prussdrv_init();
	prussdrv_open(PRU_EVTOUT_0);
	prussdrv_pruintc_init(&pruss_intc_initdata);
	unsigned int initVal = 1;
	prussdrv_pru_write_memory(PRUSS0_PRU0_DATARAM, 0, &initVal, 4);
	unsigned int sampletimestep = 10;  //delay factor (10 default, 624 for 1600 Hz)
	prussdrv_pru_write_memory(PRUSS0_PRU0_DATARAM, 1, &sampletimestep, 4);
	prussdrv_exec_program (PRU_NUM, "./pwm_test.bin");

	while(true){
		curWheelSpeedI = (int)(currentWheelSpeed*100.0);
		prussdrv_pru_write_memory(PRUSS0_PRU0_DATARAM, 0, &curWheelSpeedI, 4);
	}

	#endif
}

void ControlSteering()
{
	unsigned int curWheelSteeringI = 1;

	#ifdef TEST_PWM
	unsigned int curWheelSteeringI = 1;
	// Initialize structure used by prussdrv_pruintc_intc
	tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;
	prussdrv_init();
	prussdrv_open(PRU_EVTOUT_0);
	prussdrv_pruintc_init(&pruss_intc_initdata);
	unsigned int initVal = 1;
	prussdrv_pru_write_memory(PRUSS0_PRU0_DATARAM, 0, &initVal, 4);
	unsigned int sampletimestep = 10;  //delay factor (10 default, 624 for 1600 Hz)
	prussdrv_pru_write_memory(PRUSS0_PRU0_DATARAM, 1, &sampletimestep, 4);
	prussdrv_exec_program (PRU_NUM, "./pwm_test.bin");
	while(true){
		curWheelSteeringI = (int)(currentWheelSteering*100.0);
		prussdrv_pru_write_memory(PRUSS0_PRU0_DATARAM, 0, &curWheelSpeedI, 4);
	}
	#endif

	while(true){
		curWheelSteeringI = (int)(currentWheelSteering*100.0);
		// std::cout << curWheelSteeringI << "\n";
		usleep(1000);
	}
}

/// @todo This should not be hard-coded if a generalized model is desired.
Controller::Controller() {
	currentVehicleMode = Parser::GetControlMode();
	currentWheelSpeed = 0.0;
	currentWheelSteering = 0.5;
}

void Controller::InitializeMotorControl(){
	std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "][CTL]: Creating a thread for motor control...\n";
	std::thread runMotors(ControlMotors);
	runMotors.detach();
}

void Controller::InitializeSteeringControl(){
	std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "][CTL]: Creating a thread for steering control...\n";
	std::thread runMotors(ControlSteering);
	runMotors.detach();
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
			currentWheelSteering = 0.5;
		}else{
			g->GetCurrentGuidanceManeuver().hasFixedSpeed = true;
			currentWheelSteering = ((double)(g->GetCurrentGuidanceManeuver().turnDirection))/2.0+0.5;
			switch (g->GetCurrentGuidanceManeuver().state) {
				case ManeuverState::Calibrate:
					TimeModule::Log("CTL", "Speed fixed. Ready to calibrate.");
					TimeModule::AddMilestone("Calibration_" + std::to_string(g->GetGuidanceManeuverIndex()));
					break;
				case ManeuverState::Turn:
					TimeModule::Log("CTL", "Speed fixed. Ready to turn.");
					TimeModule::AddMilestone("Turn_" + std::to_string(g->GetGuidanceManeuverIndex()));
					break;
				case ManeuverState::Maintain:
					TimeModule::Log("CTL", "Speed fixed. Ready to maintain.");
					TimeModule::AddMilestone("Maintain_" + std::to_string(g->GetGuidanceManeuverIndex()));
					break;
				case ManeuverState::AvoidDiverge:
				  TimeModule::Log("CTL", "Speed fixed. Ready to avoid-diverge.");
					TimeModule::AddMilestone("Avoid_" + std::to_string(g->GetGuidanceManeuverIndex()));
					break;
				case ManeuverState::AvoidConverge:
					break;
				case ManeuverState::PayloadDrop:
					TimeModule::Log("CTL", "Speed fixed. Ready to drop payload.");
					TimeModule::AddMilestone("PayloadDrop_" + std::to_string(g->GetGuidanceManeuverIndex()));
					break;
			}
		}

		#ifdef SIM
		PlantModel::GetVehicle()->wheelSpeedN = currentWheelSpeed;
		PlantModel::GetVehicle()->wheelSteeringN = (currentWheelSteering-0.5)*2.0;
		#endif

		return;
	}

	#ifdef SIM
	PlantModel::GetVehicle()->wheelSpeedN = currentWheelSpeed;
	PlantModel::GetVehicle()->wheelSteeringN = (currentWheelSteering-0.5)*2.0;
	#endif

	// If the NAV plan is complete, then stop the vehicle and return->
	if (g->IsNavPlanComplete()) {

		std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "][CTL]: Nav Plan complete. Stopping vehicle.\n";
		currentWheelSpeed = 0.0;

		#ifdef TEST_PWM
		prussdrv_pru_disable(0);
		prussdrv_exit();
		#endif

		return;
	}

	// If the buffer is empty, then don't run anything.
	if (g->GetGuidanceManeuverBuffer().empty() || g->GetCurrentGuidanceManeuver().done) {
		// currentWheelSpeed = 0.0;
	}

	// If the current maneuver is a paylod drop, run payload drop functions.
	if (g->GetCurrentGuidanceManeuver().state == ManeuverState::PayloadDrop) {
		// Failsafe to ensure that the vehicle is not moving during a payload drop
		currentWheelSpeed = 0.0;
		currentWheelSteering = 0.0;
		// If we're on our last leg (return-to-home), then no need to drop payload.
		if(g->coordinateIndex == g->totalCoordinates-1 && g->GetCurrentGuidanceManeuver().done != true){
			g->GetCurrentGuidanceManeuver().payloadDropComplete = true;
			g->GetCurrentGuidanceManeuver().payloadImageTaken = true;
			std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "][CTL]: WE'RE BACK HOME! Sending fake signals back to GDE.\n";
			return;
		}

		if(g->GetCurrentGuidanceManeuver().done != true){
			PayloadDrop(g, sh);
		}

		return;
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
			std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "][CTL]: Payload deployed!\n";
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
		std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "][CTL]: Received successful image signal!\n";
	}
	else {
		std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "][CTL]: Image taking has failed.\n";
	}
	return;
}

void Controller::SetCurrentVehicleMode(VehicleMode vm) { currentVehicleMode = vm; }
VehicleMode Controller::GetCurrentVehicleMode() { return currentVehicleMode; }
void Controller::SetMaxTurnSteering(double d) { maxTurnSteering = d; }
// void Controller::SetMaxCameraAttempts(Parser::GetMaxCameraAttempts());
