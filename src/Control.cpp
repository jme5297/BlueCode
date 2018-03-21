#include <Control.h>

#ifdef TEST_PWM
#include <prussdrv.h>
#include <pruss_intc_mapping.h>
#define PRU_NUM 0
#endif

#ifdef SIM
using namespace Plant;
#endif

using namespace Guidance;
using namespace Control;
using namespace sensors;

// --------------------
// NORMALIZED
// --------------------
double norm_speed;
double norm_steer;

// --------------------
// DUTY CYCLES
// --------------------
double dutyCycle_speed; // Speed - Used for ESC
double dutyCycle_steer; // Steer - Used for vehicle wheel servo
double dutyCycle_payload; // Payload - Used for payload servo

// Is the payload servo currently active?
bool payloadServoActive;

// Has the payload servo moved for this specific payload drop?
bool hasPayloadServoMoved;

// Values for transistor
std::ofstream tReader;
std::string gpio_steer;
std::string gpio_payload;

// Prototypes for PRU running threads
void ControlMotors();
void ControlSteering();
/*!
 */
Controller::Controller() {}

/*!
 * Begin a thread for the ESC PRU
 */
void Controller::InitializeMotorControl() {
	dutyCycle_speed = Parser::GetDC_ESC_Zero();
	norm_speed = 0.0;
#ifdef TEST_PWM
	TimeModule::Log("CTL", "Creating a thread for motor control...");
	std::thread runMotors(ControlMotors);
	runMotors.detach();
#endif
}

/*!
 * Begin a thread for the Steering and Payload PRU.
 */
void Controller::InitializeSteeringControl() {
	dutyCycle_steer = Parser::GetDC_Steer_Straight();
	dutyCycle_payload = Parser::GetDC_Payload_Start();
	gpio_steer = "/sys/class/gpio/gpio" + std::to_string(Parser::GetGPIO_Steer()) + "/value";
	gpio_payload = "/sys/class/gpio/gpio" + std::to_string(Parser::GetGPIO_Payload()) + "/value";
	payloadServoActive = false;
	hasPayloadServoMoved = false;

#ifdef TEST_PWM

	// Make sure the payload transistor power is off
	tReader.open(gpio_payload);
	tReader << 0;
	tReader.close();
	tReader.open(gpio_steer);
	tReader << 1;
	tReader.close();

	TimeModule::Log("CTL", "Creating a thread for steering control...");
	std::thread runMotors(ControlSteering);
	runMotors.detach();
#endif
}

/*!
 * Main Run routine for Controller.
 * The controller first determines if a fixed speed has been acheived. If not,
 * wheel speed value is accelerated by the speed rate set in the Guider.
 *
 * Once a fixed speed has been acheived, Controller will handle special cases (such as
 * NavPlan-complete and Payload Drop scenarios).
 */
void Controller::Run(Guider* g, SensorHub* sh) {

	// Update the speeds if the speed is not yet fixed.
	if (!g->GetCurrentGuidanceManeuver().hasFixedSpeed) {

		// If we still need to change speed, then do so now, and make sure we are not turning.
		if (!(fabs(norm_speed - g->GetCurrentGuidanceManeuver().speed) <= 2.0 * g->GetCurrentGuidanceManeuver().speedRate)) {

			// Update the normalized wheel speed and make sure we're not turning.
			double sign = (g->GetCurrentGuidanceManeuver().speed - norm_speed) / fabs(norm_speed - g->GetCurrentGuidanceManeuver().speed);
			norm_speed += sign * g->GetCurrentGuidanceManeuver().speedRate;
			norm_steer = 0.0;
		}
		else {
			g->GetCurrentGuidanceManeuver().hasFixedSpeed = true;

			// Update normalized speed and steering values
			norm_speed = g->GetCurrentGuidanceManeuver().speed;
			norm_steer = (double)(g->GetCurrentGuidanceManeuver().turnDirection);

			// Begin timers for each type of maneuver.
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

		// Update the duty cycles.
		dutyCycle_speed = Parser::GetDC_ESC_Zero() + (Parser::GetDC_ESC_Fwd() - Parser::GetDC_ESC_Back()) * ( 0.5 * norm_speed );
		dutyCycle_steer = Parser::GetDC_Steer_Straight() + (Parser::GetDC_Steer_Right() - Parser::GetDC_Steer_Left()) * ( 0.5 * norm_steer );

		// Update the plant values if in sim mode.
#ifdef SIM
		PlantModel::GetVehicle()->wheelSpeedN = norm_speed;
		PlantModel::GetVehicle()->wheelSteeringN = norm_steer;
#endif

		// Don't continue operations until speed has been fixed.
		return;
	}

	// Update the plant values if in sim mode (have to do it here as well)
#ifdef SIM
	PlantModel::GetVehicle()->wheelSpeedN = norm_speed;
	PlantModel::GetVehicle()->wheelSteeringN = norm_steer;
#endif

	// If the NAV plan is complete, then stop the vehicle and return.
	if (g->IsNavPlanComplete()) {

		TimeModule::Log("CTL","Nav Plan complete signal from GDE. Stopping vehicle.");
		dutyCycle_speed = Parser::GetDC_ESC_Zero();

		// Disable and exit out of the PRU.
		// NOTE:: Make this a function.
#ifdef TEST_PWM
		// Exiting out of the motor
		tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;
		prussdrv_init();
		prussdrv_open(PRU_EVTOUT_0);
		prussdrv_pruintc_init(&pruss_intc_initdata);
		prussdrv_exec_program(PRU_NUM, "./pru1.bin");
		unsigned int mode = 0;
		prussdrv_pru_write_memory(PRUSS0_PRU0_DATARAM, 3, &mode, 4);
		// Exit out of the steering
		prussdrv_init();
		prussdrv_open(PRU_EVTOUT_1);
		prussdrv_pruintc_init(&pruss_intc_initdata);
		prussdrv_exec_program(PRU_NUM, "./pru2.bin");
		//unsigned int mode = 0;
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
		dutyCycle_speed = Parser::GetDC_ESC_Zero();
		dutyCycle_steer = Parser::GetDC_Steer_Straight();
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
			dutyCycle_payload += Parser::GetDC_Payload_Delta();

#ifdef TEST_PWM
			// Switch off the transistor for the steering servo.
			tReader.open(gpio_steer);
			tReader << 0;
			tReader.close();
#endif

			// Begin writing the payload servo duty cycle value to the PRU memory.
			payloadServoActive = true;
			hasPayloadServoMoved = true;

#ifdef TEST_PWM
			// Switch on the transistor for the payload servo.
			tReader.open(gpio_payload);
			tReader << 1;
			tReader.close();
#endif

			TimeModule::Log("CTL", "Payload Servo Active. Allowing a grace period.");
		}

		// When enough time has elapsed, set the payload dropped flag to true.
		if (TimeModule::GetElapsedTime("PayloadDrop_" + std::to_string(g->GetGuidanceManeuverIndex())) >= g->GetPayloadServoTime()) {
			g->GetCurrentGuidanceManeuver().payloadDropComplete = true;

#ifdef TEST_PWM
			// Switch off the transister for the payload servo.
			tReader.open(gpio_payload);
			tReader << 0;
			tReader.close();
#endif

			// Disable writing the payload duty cycle to the PRU memory.
			payloadServoActive = false;
			hasPayloadServoMoved = false;

#ifdef TEST_PWM
			// Switch on the transistor for the steering servo.
			tReader.open(gpio_steer);
			tReader << 1;
			tReader.close();
#endif

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

/*!
 * Main thread for controlling the ESC PRU.
 */
void ControlMotors()
{

#ifdef TEST_PWM
	// Initialize structure used by prussdrv_pruintc_intc
	tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;
	prussdrv_init();
	prussdrv_open(PRU_EVTOUT_0);
	prussdrv_pruintc_init(&pruss_intc_initdata);
	prussdrv_exec_program(PRU_NUM, "./pru1.bin");
	unsigned int delay_period = static_cast<unsigned int>(Parser::GetPRU_ESC_Delay());
	unsigned int duty_cycle = static_cast<unsigned int>(Parser::GetDC_ESC_Zero()*Parser::GetPRU_Sample_Rate());
	unsigned int mode = 1;
	//prussdrv_pru_write_memory(PRUSS0_PRU0_DATARAM, 0, &ping_val, 4);
	//prussdrv_pru_write_memory(PRUSS0_PRU0_DATARAM, 1, &duty_cycle, 4);
	prussdrv_pru_write_memory(PRUSS0_PRU0_DATARAM, 2, &delay_period, 4);
	prussdrv_pru_write_memory(PRUSS0_PRU0_DATARAM, 3, &mode, 4);

	unsigned int dutyCycle_speed_I = static_cast<unsigned int>(Parser::GetDC_ESC_Zero()*Parser::GetPRU_Sample_Rate());

	while (true) {
		dutyCycle_speed_I = static_cast<unsigned int>(dutyCycle_speed * Parser::GetPRU_Sample_Rate());
		if (dutyCycle_speed_I == 0) { dutyCycle_speed_I = 1; }
		if (dutyCycle_speed_I == static_cast<unsigned int>(Parser::GetPRU_Sample_Rate())) { dutyCycle_speed_I -= 1; }
		//std::cout << dutyCycle_speed_I << "\n";
		//first = false;
		prussdrv_pru_write_memory(PRUSS0_PRU0_DATARAM, 1, &dutyCycle_speed_I, 4);
		usleep(10000);
	}
#endif
}

/*!
 * Main thread for controlling the Steering and Payload servos.
 */
void ControlSteering()
{

#ifdef TEST_PWM
	// Initialize structure used by prussdrv_pruintc_intc
	tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;
	prussdrv_init();
	prussdrv_open(PRU_EVTOUT_1);
	prussdrv_pruintc_init(&pruss_intc_initdata);
	prussdrv_exec_program(1, "./pru2.bin");
	unsigned int delay_period = static_cast<unsigned int>(Parser::GetPRU_Steer_Delay());
	unsigned int duty_cycle = static_cast<unsigned int>(Parser::GetDC_Steer_Straight()*Parser::GetPRU_Sample_Rate());
	unsigned int mode = 1;
	//prussdrv_pru_write_memory(PRUSS0_PRU1_DATARAM, 0, &ping_val, 4);
	//prussdrv_pru_write_memory(PRUSS0_PRU1_DATARAM, 1, &duty_cycle, 4);
	prussdrv_pru_write_memory(PRUSS0_PRU1_DATARAM, 2, &delay_period, 4);
	prussdrv_pru_write_memory(PRUSS0_PRU1_DATARAM, 3, &mode, 4);

	// Check to see if we are controlling either payload or steering.
	unsigned int dutyCycle_steer_I = static_cast<unsigned int>(Parser::GetDC_Steer_Straight()*Parser::GetPRU_Sample_Rate());
	unsigned int dutyCycle_payload_I = static_cast<unsigned int>(Parser::GetDC_Payload_Start()*Parser::GetPRU_Sample_Rate());

	// Main logic loop
	while (true) {

		// Steering servo
		dutyCycle_steer_I = static_cast<unsigned int>(dutyCycle_steer * Parser::GetPRU_Sample_Rate());
		if (dutyCycle_steer_I == 0) { dutyCycle_steer_I = 1; }
		if (dutyCycle_steer_I == static_cast<unsigned int>(Parser::GetPRU_Sample_Rate())) { dutyCycle_steer_I -= 1; }

		// Payload servo
		dutyCycle_payload_I = static_cast<unsigned int>(dutyCycle_payload * Parser::GetPRU_Sample_Rate());
		if (dutyCycle_payload_I == 0) { dutyCycle_payload_I = 1; }
		if (dutyCycle_payload_I == static_cast<unsigned int>(Parser::GetPRU_Sample_Rate())) { dutyCycle_payload_I -= 1; }

		// std::cout << dutyCycle_steer << "\n";
		// Test to see if payload servo is active to determine which memory we are writing.
		if(payloadServoActive){
			prussdrv_pru_write_memory(PRUSS0_PRU1_DATARAM, 1, &dutyCycle_payload_I, 4);
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
	dutyCycle_steer = Parser::GetDC_Steer_Straight();
	while (fabs(norm_speed) > 2.0*Parser::GetAccFactorObs()*Parser::GetRefresh_GUID()) {
		norm_speed -= norm_speed / fabs(norm_speed) * Parser::GetAccFactorObs() * Parser::GetRefresh_GUID();
		dutyCycle_speed = Parser::GetDC_ESC_Zero() + (Parser::GetDC_ESC_Fwd() - Parser::GetDC_ESC_Back()) * ( 0.5 * norm_speed );
		//if (dutyCycle_speed < 0.0) { dutyCycle_speed = 0.0; }
#ifdef TEST_PWM
		usleep(100000);
#endif
	}
	std::cout << "===Deceleration complete.===\n";

#ifdef TEST_PWM
	// Exiting out of the motor
	tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;
	prussdrv_init();
	prussdrv_open(PRU_EVTOUT_0);
	prussdrv_pruintc_init(&pruss_intc_initdata);
	prussdrv_exec_program(PRU_NUM, "./pru1.bin");
	unsigned int mode = 0;
	prussdrv_pru_write_memory(PRUSS0_PRU0_DATARAM, 3, &mode, 4);

	// Exit out of the steering
	prussdrv_init();
	prussdrv_open(PRU_EVTOUT_1);
	prussdrv_pruintc_init(&pruss_intc_initdata);
	prussdrv_exec_program(PRU_NUM, "./pru2.bin");
	//unsigned int mode = 0;
	prussdrv_pru_write_memory(PRUSS0_PRU1_DATARAM, 3, &mode, 4);

#endif
}

void Controller::SetMaxTurnSteering(double d) { maxTurnSteering = d; }
// void Controller::SetMaxCameraAttempts(Parser::GetMaxCameraAttempts());
