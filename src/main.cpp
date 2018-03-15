// BlueCode Includes
#include <Parser.h>
#include <TimeModule.h>
#include <Navigation.h>
#include <Guidance.h>
#include <Control.h>

// Optional simulation include
#ifdef SIM
#include <PlantModel/PlantModel.h>
#endif

using namespace Navigation;
using namespace Guidance;
using namespace Control;
using namespace std::chrono;
using namespace Times;

#ifdef SIM
using namespace Plant;

// Required because PlantModel is an event handler for Irrlicht.
PlantModel pm;
#endif

//-----------------------------------------------------------
//                                        Function Prototypes
//-----------------------------------------------------------

/*!
 * This class provides setup procedures, reading in information
 * from data files, and initialization of al sensors.
 * @param[in] mySensorHub	- Reference to the main sensor hub
 * @param[in] myNavigator	- Reference to the main navigation class
 * @param[in] myGuider		- Reference to the main guidance class
 * @param[in] myController	- Reference to the main controls class.
 */
bool ProgramSetup(SensorHub* mySensorHub,
	Navigator* myNavigator,
	Guider* myGuider,
	Controller* myController);
/*!
 * Main operations loop of the program. This loops is ran every time-step.
 * subroutines are only called at specific frequencies. This class runs continuously
 * until specific flags are triggered from the Guidance class.
 * @param[in] mySensorHub	- Reference to the main sensor hub
 * @param[in] myNavigator	- Reference to the main navigation class
 * @param[in] myGuider		- Reference to the main guidance class
 * @param[in] myController	- Reference to the main controls class.
 */
void MainOperations(SensorHub* mySensorHub,
	Navigator* myNavigator,
	Guider* myGuider,
	Controller* myController);
/*!
 * Stores any operations required to run after the completion of the program. This could
 * pertain to ensuring the vehicle is not moving, and that the simulation window is
 * properly torn down->
 */
void CleanupOperations();
/*!
 * Tests sensor connectivity of all sensors. This sensor connectivity function is ran
 * during program setup and is used to ensure all sensors are properly reading information
 * from the respective serial ports.
 */
bool TestSensorConnectivity();
/*!
 * [Deprecated] Old method to provide nav-plan coordinates to the program setup routines. This
 * has been replaced with Config.txt for handling lat/lon target inputs. However, it's left in the
 * code just in-case manual functionality is required to be added again later.
 */
Navigator InutNavPlanCoordinates();
/*!
 * Prints Nav Pan information after the nav plan has been constructed. This is to allow the user
 * to see currently constructed/active plan in the event that changes must be made.
 */
void PrintNavPlanInfo(Navigator* n, SensorHub* sh);

// Create all of the main structures to be passed to the respective subroutines.
Navigator NAV; Navigator* myNavigator = &NAV;
Guider GUID; Guider* myGuider = &GUID;
SensorHub SH; SensorHub* mySensorHub = &SH;
Controller CTRL; Controller* myController = &CTRL;

// Handler for premature ctrl-c statements
void my_handler(int s){
		myController->EmergencyShutdown();
    exit(1);
}

//-----------------------------------------------------------
//                                           Program Entrance
//-----------------------------------------------------------
int main(int argc, char* argv[]) {

#ifdef SIM
	srand(time(0));
#endif

	struct sigaction sigIntHandler;
 	sigIntHandler.sa_handler = my_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

	// Read all of the inputs in the configuration file.
	Parser::ReadInputs("../Config.txt");

	// Initialize all sensors, test connectivity, and construct a Nav Plan->
	bool setup = ProgramSetup(mySensorHub, myNavigator, myGuider, myController);

	// If failure in setup at any given time, then program cannot continue.
	if (!setup) {
		std::cout << "Error setting up program. Exiting program.\n";
		return 0;
	}

	// Begin main operations.
	MainOperations(mySensorHub, myNavigator, myGuider, myController);

	// Allow user to provide input before closing out of the Irrlicht window.
#ifdef SIM
	//std::cout << "Press return to finish...";
	//std::cin.get();
#endif

	// Perform any necessary clean-up operations.
	CleanupOperations();

	return 0;
}

//-----------------------------------------------------------
//                                       Function Definitions
//-----------------------------------------------------------

bool ProgramSetup(SensorHub* mySensorHub, Navigator* myNavigator, Guider* myGuider, Controller* myController)
{
	// Set all other configuration parameters that were defined in Config.txt.
	myGuider->SetPayloadDropRadius(Parser::GetPayloadDropRadius());
	myGuider->SetOffAngleDeviate(Parser::GetOffAngleDeviate());
	myGuider->SetOffAngleAccepted(Parser::GetOffAngleAccepted());
	myGuider->SetCalibrationTime(Parser::GetCalibrationTime());
	myGuider->SetObstacleDivergenceTime(Parser::GetObstacleDivergenceTime());
	myGuider->SetPayloadServoTime(Parser::GetPayloadServoTime());
	myGuider->SetTurnFactorDPS(Parser::GetTurnFactorDPS());
	myGuider->SetMaxVehicleSpeed(Parser::GetMaxSpeedMPS());
	myGuider->SetMinimumMaintainTime(Parser::GetMinimumMaintainTime());
	myGuider->SetObstacleDivergenceAngle(Parser::GetObstacleDivergenceAngle());
	myController->SetCurrentVehicleMode(Parser::GetControlMode());
	myController->SetMaxTurnSteering(Parser::GetMaxTurnSteering());
	myController->SetMaxCameraAttempts(Parser::GetMaxCameraAttempts());

	// Set additional parameters if running the simulation->
#ifdef SIM
	mySensorHub->GetGPS()->SetGPSUncertainty(Parser::GetGPSUncertainty());
	PlantModel::GetVehicle()->vehicleType = Parser::GetVehicleTypeSim();
	PlantModel::GetVehicle()->heading = Parser::GetInitialHeading();
	PlantModel::GetVehicle()->gps.coords.lat = Parser::GetInitialLatitude();
	PlantModel::GetVehicle()->gps.coords.lon = Parser::GetInitialLongitude();
#endif

	// Ensure all sensors are connected.
	std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "][MNE]: Initializing sensor connections... \n";
	bool initialized = mySensorHub->InitAllSensors();
	if (!initialized) {
		std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "][MNE]: ERROR: Not all sensors have initialized. Exiting program.\n";
		return false;
	}

	// Add the waypoints provided from the configuration file, Config.txt, and
	// make sure that there was at-least one nav plan coordinate added.
	myNavigator->Initialize(mySensorHub);
	myNavigator->AddCoordinates(Parser::GetInputCoordinates());
	if (!myNavigator->IsPopulated()) {
		std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "][MNE]: ERROR: Nav Plan not properly populated. Exiting program.\n";
		return false;
	}

	// If user has specified, optimize the initial nav-plan->
	if (Parser::GetOptimize()) {
		std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "][MNE]: Requesting Nav-Plan optimizaiton.\n";
		myNavigator->ConstructNavPlan(0);
	}

	// Come up with nominal Nav-Plan movement and distance info.
	myNavigator->PopulateMovements(mySensorHub);

	std::ofstream pts;
	pts.open("pts.csv");
	std::vector<Coordinate> coords = myNavigator->GetWaypoints();
	for (int i = 0; i < (int)coords.size(); i++) {
		pts << coords[i].lon << "," << coords[i].lat << "\n";
	}
	pts.close();

	return true;
}

void MainOperations(SensorHub* mySensorHub, Navigator* myNavigator, Guider* myGuider, Controller* myController) {

	// Initialize the plant model (and Irrlicht window, if desired).
#ifdef SIM
	// Initialize is a non-static function.
	pm.Initialize();
	#ifdef USE_IRRLICHT
	PlantModel::DrawObstacles(Parser::GetObstacles());
	PlantModel::DrawPayloadLocations(myNavigator->GetNavPlan().coordinates,myGuider->GetPayloadDropRadius());
	#endif
#endif

	// If debug mode is active, ensure the TimeModule is not running on std::chrono.
#ifdef DEBUG
	TimeModule::SetTimeSimDelta(Parser::GetTimeDelta());
#endif

	// Set initial process frequencies and log the starting time of the program.
	TimeModule::AddMilestone("BeginMainOpsTime");
	TimeModule::InitProccessCounter("Nav", Parser::GetRefresh_NAV());
	TimeModule::InitProccessCounter("Guid", Parser::GetRefresh_GUID());
	TimeModule::InitProccessCounter("Ctrl", Parser::GetRefresh_CTRL());
	TimeModule::InitProccessCounter("Write", Parser::GetRefresh_OUT());
	// TimeModule::InitProccessCounter("Print", 1.0);

	// If in simulation mode, set the plant model process frequency.
#ifdef SIM
	TimeModule::InitProccessCounter("Plant", Parser::GetSimDelta());
#endif

	int ii = 0;
	std::string name = "data_" + std::to_string(ii) + ".csv";
	std::ifstream f(name.c_str());
	while(f.good()){
		ii = ii + 1;
		name = "data_" + std::to_string(ii) + ".csv";
		f.close();
		f.open(name.c_str());
	}
	f.close();

	// Open a file to save the output information the vehicle.
	std::ofstream output;
	output.open((name).c_str());


	// Initialize the motor, steering, and payload control.
	myController->InitializeMotorControl();
	//myController->InitializeSteeringControl();

	//-----------------------------------------
	//                     Main Logic loop
	//-----------------------------------------
	bool running = true;
	while (running) {

#ifdef DEBUG
		TimeModule::Run();
#endif

		// Run the plant model, and update the simulation time (if running DEBUG mode).
#ifdef SIM
		if (TimeModule::ProccessUpdate("Plant")) {

			#ifdef USE_IRRLICHT
			// Passing GPS coordinates so the simulation can plot true vs. actual.
			PlantModel::SendGPSData(
				mySensorHub->GetGPS()->GetCurrentGPSCoordinates(),
				mySensorHub->GetGPS()->GetGPSGroundCourse());
			#endif

			PlantModel::Run(TimeModule::GetLastProccessDelta("Plant"));
		}
#endif
		// Run Navigator.
		if (TimeModule::ProccessUpdate("Nav")) {
			myNavigator->Run(mySensorHub);
		}

		// Run Guider.
		if (TimeModule::ProccessUpdate("Guid")) {
			myGuider->Run(myNavigator);
		}

		// Run Controller.
		if (TimeModule::ProccessUpdate("Ctrl")) {
			myController->Run(myGuider, mySensorHub);
		}

		// std::cout << TimeModule::GetElapsedTime("BeginMainOpsTime") << std::endl;

		// If the "Print" process is intialized, then display basic information->
		double lon = mySensorHub->GetGPS()->GetCurrentGPSCoordinates().lon;
		double lat = mySensorHub->GetGPS()->GetCurrentGPSCoordinates().lat;
		#ifdef SIM
		lon = PlantModel::GetVehicle()->gps.coords.lon;
		lat = PlantModel::GetVehicle()->gps.coords.lat;
		#endif

		if (TimeModule::ProccessUpdate("Print")) {
			std::cout <<
				"t: " << std::fixed << std::setprecision(5) << TimeModule::GetElapsedTime("BeginMainOpsTime") <<
				" --- lat: " << std::setprecision(12) << myNavigator->GetCoordinates().lat <<
				", lon: " << myNavigator->GetCoordinates().lon << "\n";
		}

		// Write program information to the output data file.
		if (TimeModule::ProccessUpdate("Write")) {
			output << TimeModule::GetElapsedTime("BeginMainOpsTime") << "," << std::setprecision(12)
				<< lon << ","
				<< lat << ","
				<< myNavigator->GetHeading() << "\n";
		}

		// If the Guider determines the nav plan to be complete, then stop the main operations loop.
		if (myGuider->IsNavPlanComplete()) {
			running = false;
		}
	}

	std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "][MNE]: Main operations complete.\n";
	output.close();
	return;
}

void CleanupOperations() {
	// If the simulation is running, then drop the Irrlicht device.
#ifdef SIM
	PlantModel::Cleanup();
#endif
	return;
}

// Ensure that all connected sensors are working properly.
bool TestSensorConnectivity() {


	return true;
}

Navigator InutNavPlanCoordinates() {

	// Create a new nav plan to populate
	Navigator navigator;

	int response;
	int index = 0;
	std::cout << "Add waypoint #" + std::to_string(index) + "? (1=y,0=n): ";
	std::cin >> response;

	while (response >= 1) {

		double c1;
		double c2;

		std::cout << "Waypoint #" + std::to_string(index) + ", lon: ";
		std::cin >> c1;
		std::cout << "Waypoint #" + std::to_string(index) + ", lat: ";
		std::cin >> c2;

		// Add these two coordinates to the planned nav plan
		navigator.AddCoordinate(-1, c1, c2);

		index++;

		std::cout << "Add waypoint " + std::to_string(index) + "? (1=y,0=n,-1=redo): ";
		std::cin >> response;
	}

	// If an undo is requested, then get recursive!
	if (response == -1)
	{
		navigator = InutNavPlanCoordinates();
	}

	// Print out a list of coordinates that will be used to a CSV file.
	std::ofstream pts;
	pts.open("out/pts.csv");
	std::vector<Coordinate> coords = navigator.GetWaypoints();
	for (int i = 0; i < (int)coords.size(); i++) {
		pts << coords[i].lon << "," << coords[i].lat << "\n";
	}
	pts.close();

	return navigator;
}

void PrintNavPlanInfo(Navigator* n, SensorHub* sh) {

	std::vector<Coordinate> coords = n->GetWaypoints();
	std::vector<Movement> moves = n->GetMovements();
	Coordinate myLoc = sh->GetGPS()->GetCurrentGPSCoordinates();

	std::cout << "=======|| Current Nav-Plan ||=======\n";
	std::cout << "Current location of (" + std::to_string(myLoc.lon) + "," + std::to_string(myLoc.lat) + ").\n";

	for (int i = 0; i < (int)coords.size(); i++) {
		std::cout << "Proceed on heading " + std::to_string(moves[i].heading) + "deg for " + std::to_string(moves[i].distance) + "u until Waypoint #" + std::to_string(i) + " at (" + std::to_string(coords.at(i).lon) + "," + std::to_string(coords.at(i).lat) + ").\n";
	}

	return;
}
