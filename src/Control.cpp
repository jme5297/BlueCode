#include <Control.h>

#ifdef TEST_PWM
#include <prussdrv.h>
#include <pruss_intc_mapping.h>
#define PRU_NUM0 0
#define PRU_NUM1 1
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

// PRU Variables (to be written to PRU memory)
unsigned int dc0;
unsigned int dc1;
unsigned int dp0;
unsigned int dp1;
unsigned int modeOn = 1;
unsigned int modeOff = 0;

// Is the payload servo currently active?
bool payloadServoActive;

// Has the payload servo moved for this specific payload drop?
bool hasPayloadServoMoved;

// Values for transistor
std::ofstream tReader;
std::string gpio_steer;
std::string gpio_payload;

// Prototypes for PRU running threads
void DisablePRUs();
void ControlMotors();
void ControlSteering();
void WriteDutyCycle(int, double);

Controller::Controller() {}

/*!
 * Main Run routine for Controller.
 * The controller first determines if a fixed speed has been acheived. If not,
 * wheel speed value is accelerated by the speed rate set in the Guider.
 *
 * Once a fixed speed has been acheived, Controller will handle special cases (such as
 * NavPlan-complete and Payload Drop scenarios).
 */
void Controller::Run(Guider* g, SensorHub* sh) {

	//
	if (!g->GetCurrentGuidanceManeuver().hasFixedSpeed) {

		double accelTime = g->GetCurrentGuidanceManeuver().accelerationTime;
		norm_speed = g->GetCurrentGuidanceManeuver().speed;
		norm_steer = 0.0;

		// Begin timers for each type of maneuver.
		switch (g->GetCurrentGuidanceManeuver().state) {
		case ManeuverState::Calibrate:
			if(TimeModule::GetElapsedTime("Speed_" + std::to_string(g->GetGuidanceManeuverIndex())) >= accelTime)
			{
				g->GetCurrentGuidanceManeuver().hasFixedSpeed = true;
				TimeModule::Log("CTL", "Speed fixed. Ready to calibrate.");
				TimeModule::AddMilestone("Calibration_" + std::to_string(g->GetGuidanceManeuverIndex()));
			}
			break;
		case ManeuverState::Turn:
			if(TimeModule::GetElapsedTime("Speed_" + std::to_string(g->GetGuidanceManeuverIndex())) >= accelTime)
			{
				g->GetCurrentGuidanceManeuver().hasFixedSpeed = true;
				norm_steer = (double)(g->GetCurrentGuidanceManeuver().turnDirection);
				TimeModule::Log("CTL", "Speed fixed. Ready to turn.");
				TimeModule::AddMilestone("Turn_" + std::to_string(g->GetGuidanceManeuverIndex()));
			}
			break;
		case ManeuverState::Maintain:
			if(TimeModule::GetElapsedTime("Speed_" + std::to_string(g->GetGuidanceManeuverIndex())) >= accelTime)
			{
				g->GetCurrentGuidanceManeuver().hasFixedSpeed = true;
				TimeModule::Log("CTL", "Speed fixed. Ready to maintain.");
				TimeModule::AddMilestone("Maintain_" + std::to_string(g->GetGuidanceManeuverIndex()));
			}
			break;
		case ManeuverState::AvoidDiverge:
			if(TimeModule::GetElapsedTime("Speed_" + std::to_string(g->GetGuidanceManeuverIndex()) + "_" + std::to_string(g->GetCurrentGuidanceManeuver().avoidDivergeState)) >= accelTime)
			{
				if(g->GetCurrentGuidanceManeuver().avoidDivergeState == 3)
				{
					g->GetCurrentGuidanceManeuver().hasFixedSpeed = true;
					TimeModule::Log("CTL", "Speed fixed. Ready to avoid-maintain.");
					TimeModule::AddMilestone("Diverge_" + std::to_string(g->GetGuidanceManeuverIndex()));
				}
				else if(g->GetCurrentGuidanceManeuver().avoidDivergeState == 2)
				{
					g->GetCurrentGuidanceManeuver().hasFixedSpeed = true;
					norm_steer = 0.0;
					TimeModule::Log("CTL", "We've stopped. Ready to move forwards.");
				}
				else if(g->GetCurrentGuidanceManeuver().avoidDivergeState == 1)
				{
					g->GetCurrentGuidanceManeuver().hasFixedSpeed = true;
					norm_steer = (double)(g->GetCurrentGuidanceManeuver().turnDirection);
					TimeModule::Log("CTL", "Speed fixed. Ready to perform backwards turn.");
					TimeModule::AddMilestone("Avoid_" + std::to_string(g->GetGuidanceManeuverIndex()));
				}
				else if(g->GetCurrentGuidanceManeuver().avoidDivergeState == 0)
				{
					g->GetCurrentGuidanceManeuver().hasFixedSpeed = true;
#ifdef TEST_PWM

					TimeModule::Log("CTL", "Calibrate ESC to zero...");
					dutyCycle_speed = 0.15;
					WriteDutyCycle(0, dutyCycle_speed);
					usleep(50000);
#endif
					TimeModule::Log("CTL", "We've stopped. Ready to move backwards.");
				}
			}
			break;
		case ManeuverState::PayloadDrop:
			if(TimeModule::GetElapsedTime("Speed_" + std::to_string(g->GetGuidanceManeuverIndex())) >= accelTime)
			{
				g->GetCurrentGuidanceManeuver().hasFixedSpeed = true;
				norm_steer = (double)(g->GetCurrentGuidanceManeuver().turnDirection);
				TimeModule::Log("CTL", "Speed fixed. Ready to drop payload.");
				TimeModule::AddMilestone("PayloadDrop_" + std::to_string(g->GetGuidanceManeuverIndex()));
			}
			break;
		}

		// Update the duty cycles.
		dutyCycle_speed = Parser::GetDC_ESC_Zero() + (Parser::GetDC_ESC_Fwd() - Parser::GetDC_ESC_Back()) * ( 0.5 * norm_speed );
		dutyCycle_steer = Parser::GetDC_Steer_Straight() + (Parser::GetDC_Steer_Right() - Parser::GetDC_Steer_Left()) * ( 0.5 * norm_steer );
		WriteDutyCycle(0, dutyCycle_speed);
		WriteDutyCycle(1, dutyCycle_steer);

		// Update the plant values if in sim mode.
#ifdef SIM
		PlantModel::GetVehicle()->wheelSpeedN = norm_speed;
		PlantModel::GetVehicle()->wheelSteeringN = norm_steer;
#endif

		// Don't perform other logic until we're complete.
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
		WriteDutyCycle(0, dutyCycle_speed);

		// Disable and exit out of the PRU.
		DisablePRUs();

		return;
	}

	// If the buffer is empty, then don't run anything.
	// Note: This is not needed now, but will be left in until certain to not be used.
	if (g->GetGuidanceManeuverBuffer().empty() || g->GetCurrentGuidanceManeuver().done) {

	}

	// If the current maneuver is a paylod drop, run payload drop functions.
	if (g->GetCurrentGuidanceManeuver().state == ManeuverState::PayloadDrop) {
		// Failsafe to ensure that the vehicle is not moving during a payload drop
		dutyCycle_speed = Parser::GetDC_ESC_Zero();
		WriteDutyCycle(0, dutyCycle_speed);

		// If we're on our last leg (return-to-home), then no need to drop payload.
		if (g->coordinateIndex == g->totalCoordinates - 1 && g->GetCurrentGuidanceManeuver().done != true) {
			g->GetCurrentGuidanceManeuver().payloadDropComplete = true;
			g->GetCurrentGuidanceManeuver().payloadImageTaken = true;
			TimeModule::Log("CTL","WE'RE BACK HOME! Sending fake signals back to GDE.");
			DisablePRUs();
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
			usleep(10000);
#endif

			// Begin writing the payload servo duty cycle value to the PRU memory.
			hasPayloadServoMoved = true;
			WriteDutyCycle(1, dutyCycle_payload);
			usleep(10000);

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
			usleep(10000);
#endif

			// Disable writing the payload duty cycle to the PRU memory.
			hasPayloadServoMoved = false;
			WriteDutyCycle(1, dutyCycle_steer);
			usleep(10000);

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
	if (TimeModule::GetElapsedTime("FixSteering_" + std::to_string(g->GetGuidanceManeuverIndex())) >= 0.5) {
			TimeModule::Log("CTL", "Steering servo fixed back to position.");
	}else{
		return;
	}

	// Take an image
	int attempts = 0;
	for(attempts = 0; attempts < Parser::GetMaxCameraAttempts(); attempts++){
		// Attempt to take an image after the payload has been dropped.
		bool imageTaken = sh->GetCamera()->TakeImage(g->GetCurrentGuidanceManeuver().index);
		if (imageTaken) {
			g->GetCurrentGuidanceManeuver().payloadImageTaken = true;
			TimeModule::Log("CTL","Received successful image signal after " + std::to_string(attempts+1) + " attempts.");
			break;
		}
		else {
			TimeModule::Log("CTL","Image taking has failed on attempt " + std::to_string(attempts+1) + ". ");
#ifdef USE_CAMERA
			usleep(10000);
#endif
		}
	}

	if(attempts == Parser::GetMaxCameraAttempts()){
		TimeModule::Log("CTL", "I give up!");
		g->GetCurrentGuidanceManeuver().payloadImageTaken = true;
	}
	return;
}

/*!
 * Write a new duty cycle to a specific PRU.
 */
void WriteDutyCycle(int pru, double dc){
#ifdef TEST_PWM
	if(pru == 0){
		dc0 = static_cast<unsigned int>(dc * Parser::GetPRU_Sample_Rate());
		if(dc0 == static_cast<unsigned int>(Parser::GetPRU_Sample_Rate())){ dc0 -= 1; }
		prussdrv_pru_write_memory(PRUSS0_PRU0_DATARAM, 1, &dc0, 4);
	}else{
//		std::cout << dc << "\n";
		dc1 = static_cast<unsigned int>(dc * Parser::GetPRU_Sample_Rate());
		if(dc1 == static_cast<unsigned int>(Parser::GetPRU_Sample_Rate())){ dc1 -= 1; }
		prussdrv_pru_write_memory(PRUSS0_PRU1_DATARAM, 1, &dc1, 4);
	}
#endif
}

/*!
 * Begin a thread for the ESC PRU
 */
void Controller::InitializeMotorControl() {
	dutyCycle_speed = Parser::GetDC_ESC_Zero();
	dc0 = static_cast<unsigned int>(dutyCycle_speed * Parser::GetPRU_Sample_Rate());
	dp0 = static_cast<unsigned int>(Parser::GetPRU_ESC_Delay());
	norm_speed = 0.0;
#ifdef TEST_PWM
	TimeModule::Log("CTL", "Initializing motor control.");
	ControlMotors();
#endif
}

/*!
 * Begin a thread for the Steering and Payload PRU.
 */
void Controller::InitializeSteeringControl() {
	dutyCycle_steer = Parser::GetDC_Steer_Straight();
	dutyCycle_payload = Parser::GetDC_Payload_Start();
	dc1 = static_cast<unsigned int>(dutyCycle_steer * Parser::GetPRU_Sample_Rate());
	dp1 = static_cast<unsigned int>(Parser::GetPRU_Steer_Delay());
	norm_steer = 0.0;
	gpio_steer = "/sys/class/gpio/gpio" + std::to_string(Parser::GetGPIO_Steer()) + "/value";
	gpio_payload = "/sys/class/gpio/gpio" + std::to_string(Parser::GetGPIO_Payload()) + "/value";
	payloadServoActive = false;
	hasPayloadServoMoved = false;

#ifdef TEST_PWM
	// Make sure the payload transistor power is off, and steering power is on
	tReader.open(gpio_payload);
	tReader << 0;
	tReader.close();
	tReader.open(gpio_steer);
	tReader << 1;
	tReader.close();
	TimeModule::Log("CTL", "Initializing steering/payload control.");
	ControlSteering();
#endif
}

/*!
 * Main thread for controlling the ESC PRU.
 */
void ControlMotors()
{

#ifdef TEST_PWM
	TimeModule::Log("CTL", "Enabling PRU1 (Steering & Payload).");
	tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;
	prussdrv_init();
	prussdrv_open(PRU_EVTOUT_0);
	prussdrv_pruintc_init(&pruss_intc_initdata);
	prussdrv_pru_write_memory(PRUSS0_PRU0_DATARAM, 1, &dc0, 4);
	prussdrv_pru_write_memory(PRUSS0_PRU0_DATARAM, 2, &dp0, 4);
	prussdrv_pru_write_memory(PRUSS0_PRU0_DATARAM, 3, &modeOn, 4);
	prussdrv_exec_program(PRU_NUM0, "./pru1.bin");
#endif
}

/*!
 * Main thread for controlling the Steering and Payload servos.
 */
void ControlSteering()
{
#ifdef TEST_PWM
	TimeModule::Log("CTL", "Enabling PRU1 (Steering & Payload).");
	prussdrv_init();
	tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;
	prussdrv_open(PRU_EVTOUT_1);
	prussdrv_pruintc_init(&pruss_intc_initdata);
	prussdrv_pru_write_memory(PRUSS0_PRU1_DATARAM, 1, &dc1, 4);
	prussdrv_pru_write_memory(PRUSS0_PRU1_DATARAM, 2, &dp1, 4);
	prussdrv_pru_write_memory(PRUSS0_PRU1_DATARAM, 3, &modeOn, 4);
	prussdrv_exec_program(PRU_NUM1, "./pru2.bin");
#endif
}

/*!
 * Decelerate and disable PRU on vehicle during an emergency.
 * A user can trigger this function by CTRL-C at runtime.
 */
void Controller::EmergencyShutdown() {

	std::cout << "===EMERGENCY! DECELERATING!===\n";
	dutyCycle_steer = Parser::GetDC_Steer_Straight();
	WriteDutyCycle(1, dutyCycle_steer);
	dutyCycle_speed = Parser::GetDC_ESC_Zero();
	WriteDutyCycle(0, dutyCycle_speed);

#ifdef TEST_PWM
		usleep(10000);
#endif

	std::cout << "===Deceleration complete.===\n";

	DisablePRUs();
}

void DisablePRUs(){
	#ifdef TEST_PWM
		// Exiting out of the motor
		prussdrv_pru_write_memory(PRUSS0_PRU0_DATARAM, 3, &modeOff, 4);
		prussdrv_exec_program(PRU_NUM0, "./pru1.bin");

		usleep(1000);

		// Exit out of the steering
		prussdrv_pru_write_memory(PRUSS0_PRU1_DATARAM, 3, &modeOff, 4);
		prussdrv_exec_program(PRU_NUM1, "./pru2.bin");

		tReader.open(gpio_steer);
		tReader << 0;
		tReader.close();
		tReader.open(gpio_payload);
		tReader << 0;
		tReader.close();

	#endif
}

void Controller::SetMaxTurnSteering(double d) { maxTurnSteering = d; }
// void Controller::SetMaxCameraAttempts(Parser::GetMaxCameraAttempts());
