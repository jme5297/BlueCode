#include <PlantModel/PlantModel.h>
#include <Navigation.h>
#include <Controller.h>
#include <iostream>
#include <fstream>
#include <chrono>

using namespace Navigation;
using namespace Control;
using namespace Plant;
using namespace std::chrono;

// FUNCTION PROTOTYPES
void InputNavPlan();
bool ProgramSetup(NavPlanner& myNavPlanner,
									SensorHub& mySensorHub,
									Controller& myController);
void MainOperations(NavPlanner& myNavPlanner,
									SensorHub& mySensorHub,
									Controller& myController);
void CleanupOperations();
bool TestSensorConnectivity();
NavPlanner InputNavPlanCoordinates();
void PrintNavPlanInfo(NavPlanner& np, SensorHub& sh);

// Generic time counters
time_point<system_clock> beginMainOpsTime;
time_point<system_clock> lastPrintTime;
time_point<system_clock> lastWriteTime;
time_point<system_clock> lastPlantStep;
time_point<system_clock> lastNavStep;
time_point<system_clock> lastControlStep;
double dtPrint = 1.0;
double dtWrite = 0.001;
double dtPlant = 0.001;
double dtNav = 0.1;			// Update Nav on 10 Hz cycle
double dtControl = 0.1;	// Update control on 10 Hz cycle

// MAIN LOGIC ENTRANCE
int main(){

	// Create all of the structures that we'll need.
	NavPlanner myNavPlanner;
	SensorHub mySensorHub;
	Controller myController;

	#ifdef SIM
	PlantModel::Initialize();
	#endif

	// ProgramSetup handles constructing the nav plan, and ensuring
	// that all sensors are connected. This will return true if setup has finished correctly.
	bool setup = ProgramSetup(myNavPlanner, mySensorHub, myController);

	// If failure in setup at any given time, then program cannot continue.
	if(!setup){
		std::cout << "Error setting up program. Exiting program.\n";
		return 0;
	}else{
		std::cout << "Nav Plan constructed successfully!\n\n";
		PrintNavPlanInfo(myNavPlanner, mySensorHub);
	}

	// Begin main operations now.
	beginMainOpsTime = std::chrono::system_clock::now();
	MainOperations(myNavPlanner, mySensorHub, myController);
	CleanupOperations();

	return 0;
}

bool ProgramSetup(NavPlanner& myNavPlanner, SensorHub& mySensorHub, Controller& myController)
{
	std::cout << "\nHello!\n\n" << std::endl;

	// Ensure all sensors are connected. If not, exit program.
	std::cout << "Initializing sensor connections... \n";
	if(!mySensorHub.InitAllSensors()){
		std::cout << "ERROR: Not all sensors have initialized. Exiting program.\n";
		return false;
	}else{
		std::cout << "Connection with sensors established!\n";
	}

	// Construct our navigation plan once we know sensors are connected.
	std::cout << "\n===WAYPOINT INPUT===\n";
	myNavPlanner = InputNavPlanCoordinates();

	// Make sure that there was at-least one nav plan coordinate added.
	if(!myNavPlanner.IsPopulated()){
		std::cout << "ERROR: Nav Plan not properly populated. Exiting program.\n";
		return false;
	}

	// Construct the nav plan based on the waypoints provided.
	std::cout << "\n===NAV PLAN CONSTRUCTION===\n";
	myNavPlanner.ConstructNavPlan(mySensorHub);

	// Make sure the nav plan was properly constructed.
	if(!myNavPlanner.IsConstructed()){
		std::cout << "ERROR: Nav Plan not properly constructed. Exiting program.\n";
		return false;
	}else{
		myNavPlanner.PopulateMovements(mySensorHub);
	}

	return true;
}

void MainOperations(NavPlanner& myNavPlanner, SensorHub& mySensorHub, Controller& myController){

	std::cout << "Running...\n";

	lastPrintTime = std::chrono::system_clock::now();
	lastWriteTime = std::chrono::system_clock::now();
	lastPlantStep = std::chrono::system_clock::now();
	lastNavStep = std::chrono::system_clock::now();
	lastControlStep = std::chrono::system_clock::now();

	#ifdef SIM
	std::ofstream output;
	output.open("out.csv");
	// Generic testing
	PlantModel::GetVehicle()->heading = -180.0;
	myController.SetMotorLSpeed(1.0);
	myController.SetMotorRSpeed(1.0);
	#endif

	// Generic dt variable to use
	std::chrono::duration<double> dt;

	// Main logic loop - I EXPECT TO BE HERE FOR A WHILE
	bool running = true;
	while(running){

		#ifdef SIM
		dt = duration_cast<seconds>(system_clock::now() - lastPlantStep);
		if(dt.count() >= dtPlant){
			PlantModel::Run(dt.count());
			lastPlantStep = std::chrono::system_clock::now();
		}
		#endif

		// Run guidance and Navigation (and pass the controller)
		dt = std::chrono::system_clock::now() - lastNavStep;
		if(dt.count() >= dtNav){
			myNavPlanner.Run(mySensorHub, myController);
			lastNavStep = std::chrono::system_clock::now();
		}

		// Run controls
		dt = std::chrono::system_clock::now() - lastControlStep;
		if(dt.count() >= dtControl){
			myController.Run();
			lastControlStep = std::chrono::system_clock::now();
		}

		double lon = mySensorHub.GetGPS().GetCurrentGPSCoordinates().lon;
		double lat = mySensorHub.GetGPS().GetCurrentGPSCoordinates().lat;

		// Print info to screen
		dt = std::chrono::system_clock::now() - lastPrintTime;
		if(dt.count() > dtPrint){
			PlantModel::PrintStatus();
			lastPrintTime = std::chrono::system_clock::now();
		}

		// Write plant model info to a file
		dt = std::chrono::system_clock::now() - lastWriteTime;
		if(dt.count() > dtWrite){
			output << PlantModel::GetElapsedSeconds() << ","
				<< lon << ","
				<< lat << ","
				<< PlantModel::GetVehicle()->heading << "\n";
			lastWriteTime = std::chrono::system_clock::now();
		}

		// Stop sim after a certain amount of time
		#ifdef SIM
		std::chrono::duration<double> elapsed = std::chrono::system_clock::now() - beginMainOpsTime;
		double runTime = elapsed.count();
		if(runTime >= 10.0){ running = false; }
		#endif

		// Determine if we're all done here.
		if(myNavPlanner.IsNavPlanComplete() && myController.GetCurrentControlMove().done){
			std::cout << "COMPLETE!\n";
			running = false;
		}
	}

	std::cout << "Main operations complete.\n";
	output.close();

	// std::cout << ">>>done>>>  " << PlantModel::GetVehicle()->gps.coords.lat << "\n";
	return;
}

void CleanupOperations(){

	return;
}

// Ensure that all connected sensors are working properly.
bool TestSensorConnectivity(){


	return true;
}

NavPlanner InputNavPlanCoordinates(){

	// Create a new nav plan to populate
	NavPlanner navPlan;

	// -----
	// THE FOLLOWING CODE WORKS. For now, just testing with debug cases.
	// -----

	// Currently, only works with 2-D coordinates. This would most likely be
	// fine considering small angle approximations.

	int response;
	int index = 0;
	std::cout << "Add waypoint #" + std::to_string(index) + "? (1=y,0=n): ";
	std::cin >> response;

	while(response >= 1){

		double c1;
		double c2;

		std::cout << "Waypoint #" + std::to_string(index) + ", lon: ";
		std::cin >> c1;
		std::cout << "Waypoint #" + std::to_string(index) + ", lat: ";
		std::cin >> c2;

		// Add these two coordinates to the planned nav plan
		navPlan.AddCoordinate(-1, c1, c2);

		index++;

		std::cout << "Add waypoint " + std::to_string(index) + "? (1=y,0=n,-1=redo): ";
		std::cin >> response;
	}

	// If an undo is requested, then get recursive!
	if(response == -1)
	{
		navPlan = InputNavPlanCoordinates();
	}

	// Debugging case 1 - generic test
	/*
	navPlan.AddCoordinate(-1, 1.0, -21.0);
	navPlan.AddCoordinate(-1, 11.0, 12.0);
	navPlan.AddCoordinate(-1, 12.0, 2.0);
	navPlan.AddCoordinate(-1, 10.0, 0.0);
	navPlan.AddCoordinate(-1, -3.0, 3.0);
	navPlan.AddCoordinate(-1, 9.0, 5.0);
	*/

	// Debugging case 2 - stress test
	/*
	navPlan.AddCoordinate(-1, 1.0, 1.0);
	navPlan.AddCoordinate(-1, -2.0, 10.0);
	navPlan.AddCoordinate(-1, 2.0, 2.0);
	navPlan.AddCoordinate(-1, 9.0, -1.0);
	navPlan.AddCoordinate(-1, -6.0, 3.0);
	navPlan.AddCoordinate(-1, 8.0, -8.0);
	navPlan.AddCoordinate(-1, 4.0, 0.0);
	navPlan.AddCoordinate(-1, -3.0, 7.0);
	navPlan.AddCoordinate(-1, 5.0, 6.0);
	navPlan.AddCoordinate(-1, 6.0, 1.0);
	navPlan.AddCoordinate(-1, -2.0, 6.0);
	//navPlan.AddCoordinate(-1, 10.0, 6.0);
	*/

	// Debugging case 3 - heading test and patterns
	/*
	navPlan.AddCoordinate(-1, 1.0, 1.0);
	navPlan.AddCoordinate(-1, -2.0, 2.0);
	navPlan.AddCoordinate(-1, -3.0, -3.0);
	navPlan.AddCoordinate(-1, 4.0, -4.0);
	navPlan.AddCoordinate(-1, 5.0, 5.0);
	navPlan.AddCoordinate(-1, -6.0, 6.0);
	navPlan.AddCoordinate(-1, -7.0, -7.0);
	navPlan.AddCoordinate(-1, 8.0, -8.0);
	*/

	return navPlan;
}

void PrintNavPlanInfo(NavPlanner& np, SensorHub& sh){

	std::vector<Coordinate> coords = np.GetWaypoints();
	std::vector<Movement> moves = np.GetMovements();
	Coordinate myLoc = sh.GetGPS().GetCurrentGPSCoordinates();

	std::cout << "===CURRENT NAV PLAN===\n";
	std::cout << "Current location of (" + std::to_string(myLoc.lon) + "," + std::to_string(myLoc.lat) + ").\n";

	for(int i = 0; i < (int)coords.size(); i++){
		//std::cout << std::to_string(moves[i].distance) << "\n";
		std::cout << "Proceed on heading " + std::to_string(moves[i].heading) + "deg for " + std::to_string(moves[i].distance) + "u until Waypoint #" + std::to_string(i) + " at (" + std::to_string(coords.at(i).lon) + "," + std::to_string(coords.at(i).lat) + ").\n";
	}

	return;
}
