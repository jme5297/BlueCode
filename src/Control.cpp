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

// --------------------
//     DUTY CYCLES
// --------------------
double dutyCycle_speed;  		// Speed - Used for ESC
double dutyCycle_steer;			// Steer - Used for vehicle wheel servo
double dutyCycle_payload;		// Payload - Used for payload servo

// Is the payload servo currently active?
bool payloadServoActive;

// Has the payload servo moved for this specific payload drop?
bool hasPayloadServoMoved;

// Prototypes for PRU running threads
void ControlMotors();
void ControlSteering();

/*!
 * Constructor for the Controller class
 */
Controller::Controller() {
	currentVehicleMode = Parser::GetControlMode();
	dutyCycle_speed = 0.0;
	dutyCycle_steer = 0.5;
	dutyCycle_payload = 0.05;
	payloadServoActive = false;
	hasPayloadServoMoved = false;
}

/*!
 * Begin a thread for the ESC PRU
 */
void Controller::InitializeMotorControl() {
	TimeModule::Log("CTL", "Creating a thread for motor control...");
	std::thread runMotors(ControlMotors);
	runMotors.detach();
}

/*!
 * Begin a thread for the Steering and Payload PRU.
 */
void Controller::InitializeSteeringControl() {
	TimeModule::Log("CTL", "Creating a thread for steering control...");
	std::thread runMotors(ControlSteering);
	runMotors.detach();
}

/*!
 * Main Run routine for Controller.
 * The controller first determines if a fixed speed has been acheived. If not,
 * wheel speed value is accelerated by the speed rate set in the Guider. Once
 * a fixed speed has been acheived, Controller will handle special cases (such as
 * NavPlan-complete and Payload Drop scenarios).
 */
void Controller::Run(Guider* g, SensorHub* sh) {

	// Update the speeds.
	if (!g->GetCurrentGuidanceManeuver().hasFixedSpeed) {
		if (!(fabs(dutyCycle_speed - g->GetCurrentGuidanceManeuver().speed) <= 2.0 * 0.005)) {
			double sign = (g->GetCurrentGuidanceManeuver().speed - dutyCycle_speed) / fabs(dutyCycle_speed - g->GetCurrentGuidanceManeuver().speed);
			dutyCycle_speed += sign * g->GetCurrentGuidanceManeuver().speedRate;
			dutyCycle_steer = 0.5;
		}
		else {
			g->GetCurrentGuidanceManeuver().hasFixedSpeed = true;
			dutyCycle_steer = ((double)(g->GetCurrentGuidanceManeuver().turnDirection)) / 2.0 + 0.5;
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
		PlantModel::GetVehicle()->wheelSpeedN = dutyCycle_speed;
		PlantModel::GetVehicle()->wheelSteeringN = (dutyCycle_steer - 0.5)*2.0;
#endif

		return;
	}

#ifdef SIM
	PlantModel::GetVehicle()->wheelSpeedN = dutyCycle_speed;
	PlantModel::GetVehicle()->wheelSteeringN = (dutyCycle_steer - 0.5)*2.0;
#endif

	// If the NAV plan is complete, then stop the vehicle and return.
	if (g->IsNavPlanComplete()) {

		TimeModule::Log("CTL","Nav Plan complete signal from GDE. Stopping vehicle.");
		dutyCycle_speed = 0.0;

		// Disable and exit out of the PRU.
		// NOTE:: Make this a function.
#ifdef TEST_PWM
		// Exiting out of the motor
		tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;
		prussdrv_init();
		prussdrv_open(PRU_EVTOUT_0);
		prussdrv_pruintc_init(&pruss_intc_initdata);
		prussdrv_exec_program(PRU_NUM, "./pwm_final.bin");
		unsigned int mode = 0;
		prussdrv_pru_write_memory(PRUSS0_PRU0_DATARAM, 3, &mode, 4);
		// Exit out of the steering
		prussdrv_init();
		prussdrv_open(PRU_EVTOUT_1);
		prussdrv_pruintc_init(&pruss_intc_initdata);
		prussdrv_exec_program(PRU_NUM, "./pwm_final_pru2.bin");
		unsigned int mode = 0;
		prussdrv_pru_write_memory(PRUSS0_PRU1_DATARAM, 3, &mode, 4);
#endif

		return;
	}

	// If the buffer is empty, then don't run anything.
	if (g->GetGuidanceManeuverBuffer().empty() || g->GetCurrentGuidanceManeuver().done) {
		// dutyCycle_speed = 0.0;
	}

	// If the current maneuver is a paylod drop, run payload drop functions.
	if (g->GetCurrentGuidanceManeuver().state == ManeuverState::PayloadDrop) {
		// Failsafe to ensure that the vehicle is not moving during a payload drop
		dutyCycle_speed = 0.0;
		dutyCycle_steer = 0.5;
		// If we're on our last leg (return-to-home), then no need to drop payload.
		if (g->coordinateIndex == g->totalCoordinates - 1 && g->GetCurrentGuidanceManeuver().done != true) {
			g->GetCurrentGuidanceManeuver().payloadDropComplete = true;
			g->GetCurrentGuidanceManeuver().payloadImageTaken = true;
			TimeModule::Log("CTL","WE'RE BACK HOME! Sending fake signals back to GDE.");
			return;
		}

		// If the payload drop has not yet been completed, run payload drop logic.
		if (g->GetCurrentGuidanceManeuver().done != true) {
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

	// Check if the payload drop has not been completed
	if (!g->GetCurrentGuidanceManeuver().payloadDropComplete) {

		// Check to see if we have updated the payload servo value for this payload drop yet
		if(!hasPayloadServoMoved){

			// Update the current value for the payload servo. NOTE: this
			// has to be updated before setting payloadServoActive flag.
			dutyCycle_payload += Parser::GetPayloadDropMovementFactor();
			hasPayloadServoMoved = true;

			// Begin writing the payload servo duty cycle value to the PRU memory.
			payloadServoActive = true;

			// Switch on the transistor for the payload servo.
			// ----------

			TimeModule::Log("CTL", "Payload Servo Active. Allowing a grace period.");
		}

		// When enough time has elapsed, set the payload dropped flag to true.
		if (TimeModule::GetElapsedTime("PayloadDrop_" + std::to_string(g->GetGuidanceManeuverIndex())) >= g->GetPayloadServoTime()) {
			g->GetCurrentGuidanceManeuver().payloadDropComplete = true;

			// Switch off the transister for the payload servo.
			// ---------

			// Disable writing the payload duty cycle to the PRU memory.
			hasPayloadServoMoved = false;
			payloadServoActive = false;

			TimeModule::Log("CTL", "Grace period complete. Payload servo disabled.");
			TimeModule::Log("CTL", "Allowing steering servo to fix itself.");
			TimeModule::AddMilestone("FixSteering_" + std::to_string(g->GetGuidanceManeuverIndex()));
		}

		// Don't take any images until we've dropped our payload.
		return;
	}

	// Once payload drop is complete, realign the steering servo with a grace period.
	// Payload drop logic cannot continue until steering servo is fixed again.
	if (TimeModule::GetElapsedTime("FixSteering_" + std::to_string(g->GetGuidanceManeuverIndex())) >= 2.0) {
			TimeModule::Log("CTL", "Steering servo fixed back to position.");
	}else{
		return;
	}

	// Attempt to take an image after the payload has been dropped.
	bool imageTaken = sh->GetCamera()->TakeImage(g->GetCurrentGuidanceManeuver().index);
	if (imageTaken) {
		g->GetCurrentGuidanceManeuver().payloadImageTaken = true;
		TimeModule::Log("CTL","Received successful image signal!");
	}
	else {
		TimeModule::Log("CTL","Image taking has failed.");
	}
	return;
}

void ControlMotors()
{

#ifdef TEST_PWM
	// Initialize structure used by prussdrv_pruintc_intc
	tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;
	prussdrv_init();
	prussdrv_open(PRU_EVTOUT_0);
	prussdrv_pruintc_init(&pruss_intc_initdata);
	prussdrv_exec_program(PRU_NUM, "./pwm_final.bin");
	unsigned int delay_period = 624;
	unsigned int ping_val = 1;
	unsigned int duty_cycle = 1;
	unsigned int mode = 1;
	prussdrv_pru_write_memory(PRUSS0_PRU0_DATARAM, 0, &ping_val, 4);
	prussdrv_pru_write_memory(PRUSS0_PRU0_DATARAM, 1, &duty_cycle, 4);
	prussdrv_pru_write_memory(PRUSS0_PRU0_DATARAM, 2, &delay_period, 4);
	prussdrv_pru_write_memory(PRUSS0_PRU0_DATARAM, 3, &mode, 4);

	unsigned int dutyCycle_speed_I = 1;
	while (true) {
		dutyCycle_speed_I = static_cast<unsigned int>(dutyCycle_speed*100.0);
		if (dutyCycle_speed_I == 0) { dutyCycle_speed_I = 1; }
		if (dutyCycle_speed_I == 100) { dutyCycle_speed_I = 99; }
		first = false;
		prussdrv_pru_write_memory(PRUSS0_PRU0_DATARAM, 1, &dutyCycle_speed_I, 4);
		usleep(10000);
	}
#endif
}

void ControlSteering()
{

#ifdef TEST_PWM
	// Initialize structure used by prussdrv_pruintc_intc
	tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;
	prussdrv_init();
	prussdrv_open(PRU_EVTOUT_1);
	prussdrv_pruintc_init(&pruss_intc_initdata);
	prussdrv_exec_program(1, "./pwm_final_pru2.bin");
	unsigned int delay_period = 624;
	unsigned int ping_val = 1;
	unsigned int duty_cycle = 1;
	unsigned int mode = 1;
	prussdrv_pru_write_memory(PRUSS0_PRU1_DATARAM, 0, &ping_val, 4);
	prussdrv_pru_write_memory(PRUSS0_PRU1_DATARAM, 1, &duty_cycle, 4);
	prussdrv_pru_write_memory(PRUSS0_PRU1_DATARAM, 2, &delay_period, 4);
	prussdrv_pru_write_memory(PRUSS0_PRU1_DATARAM, 3, &mode, 4);

	// Check to see if we are controlling either payload or steering.
	unsigned int dutyCycle_steer_I = 1;
	unsigned int dutyCycle_payload_I = (unsigned int)(dutyCycle_payload*100.0);

	// Main logic loop
	while (true) {

		// Steering servo
		dutyCycle_steer_I = (unsigned int)(dutyCycle_steer*100.0);
		if (dutyCycle_steer_I == 0) { dutyCycle_steer_I = 1; }
		if (dutyCycle_steer_I == 100) { dutyCycle_steer_I = 99; }

		// Payload servo
		dutyCycle_payload_I = (unsigned int)(dutyCycle_payload*100.0);
		if (dutyCycle_payload_I == 0) { dutyCycle_payload_I = 1; }
		if (dutyCycle_payload_I == 100) { dutyCycle_payload_I = 99; }

		// Test to see if payload servo is active to determine which memory we are writing.
		if(payloadServoActive){
			prussdrv_pru_write_memory(PRUSS0_PRU1_DATARAM, 1, &dutyCycle_payload_I, 4);
			std::cout << dutyCycle_payload_I << "\n";
		}
		else {
			prussdrv_pru_write_memory(PRUSS0_PRU1_DATARAM, 1, &dutyCycle_steer_I, 4);
		}

		usleep(10000);
	}
#endif
}

/*!
 * Decelerate and disable PRU on vehicle during an emergency.
 * A user can trigger this function by CTRL-C at runtime.
 */
void Controller::EmergencyShutdown() {

	std::cout << "===EMERGENCY! DECELERATING!===\n";
	dutyCycle_steer = 0.0;
	while (dutyCycle_speed > 0.0) {
		dutyCycle_speed -= 0.5*Parser::GetRefresh_GUID();
		if (dutyCycle_speed < 0.0) { dutyCycle_speed = 0.0; }
#ifdef TEST_PWM
		usleep(10000);
#endif
	}
	std::cout << "===Deceleration complete.===\n";

#ifdef TEST_PWM
	// Exiting out of the motor
	tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;
	prussdrv_init();
	prussdrv_open(PRU_EVTOUT_0);
	prussdrv_pruintc_init(&pruss_intc_initdata);
	prussdrv_exec_program(PRU_NUM, "./pwm_final.bin");
	unsigned int mode = 0;
	prussdrv_pru_write_memory(PRUSS0_PRU0_DATARAM, 3, &mode, 4);

	// Exit out of the steering
	prussdrv_init();
	prussdrv_open(PRU_EVTOUT_1);
	prussdrv_pruintc_init(&pruss_intc_initdata);
	prussdrv_exec_program(PRU_NUM, "./pwm_final_pru2.bin");
	unsigned int mode = 0;
	prussdrv_pru_write_memory(PRUSS0_PRU1_DATARAM, 3, &mode, 4);

#endif
}

void Controller::SetCurrentVehicleMode(VehicleMode vm) { currentVehicleMode = vm; }
VehicleMode Controller::GetCurrentVehicleMode() { return currentVehicleMode; }
void Controller::SetMaxTurnSteering(double d) { maxTurnSteering = d; }
// void Controller::SetMaxCameraAttempts(Parser::GetMaxCameraAttempts());
