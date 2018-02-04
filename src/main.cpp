#include <PlantModel/PlantModel.h>
#include <Navigation.h>
#include <Guidance.h>
#include <Control.h>
#include <TimeModule.h>
#include <iostream>
#include <fstream>
#include <chrono>

using namespace Navigation;
using namespace Guidance;
using namespace Control;
using namespace Plant;
using namespace std::chrono;
using namespace Times;

// FUNCTION PROTOTYPES
void InutNavPlan();
bool ProgramSetup(SensorHub& mySensorHub,
									Navigator& myNavigator,
									Guider& myGuider,
									Controller& myController);
void MainOperations(SensorHub& mySensorHub,
									Navigator& myNavigator,
									Guider& myGuider,
									Controller& myController,
									TimeModule& tm);
void CleanupOperations();
bool TestSensorConnectivity();
Navigator InutNavPlanCoordinates();
void PrintNavPlanInfo(Navigator& n, SensorHub& sh);

// MAIN LOGIC ENTRANCE
int main(){

	// Create all of the structures that we'll need.
	Navigator myNavigator;
	Guider myGuider;
	SensorHub mySensorHub;
	Controller myController;
	TimeModule tm;

	#ifdef SIM
	PlantModel::Initialize();
	#ifdef DEBUG
	tm.SetTimeSimDelta(0.0001);
	#endif
	#endif

	// ProgramSetup handles constructing the nav plan, and ensuring
	// that all sensors are connected. This will return true if setup has finished correctly.
	bool setup = ProgramSetup(mySensorHub, myNavigator, myGuider, myController);

	// If failure in setup at any given time, then program cannot continue.
	if(!setup){
		std::cout << "Error setting up program. Exiting program.\n";
		return 0;
	}else{
		std::cout << "Nav Plan constructed successfully!\n\n";
		PrintNavPlanInfo(myNavigator, mySensorHub);
	}

	// Begin main operations now.
	tm.AddMilestone("BeginMainOpsTime");
	MainOperations(mySensorHub, myNavigator, myGuider, myController, tm);
	CleanupOperations();

	return 0;
}

bool ProgramSetup(SensorHub& mySensorHub, Navigator& myNavigator, Guider& myGuider, Controller& myController)
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
	std::cout << "\n===WAYPOINT InUT===\n";
	myNavigator = InutNavPlanCoordinates();

	// Make sure that there was at-least one nav plan coordinate added.
	if(!myNavigator.IsPopulated()){
		std::cout << "ERROR: Nav Plan not properly populated. Exiting program.\n";
		return false;
	}

	// Construct the nav plan based on the waypoints provided.
	std::cout << "\n===NAV PLAN CONSTRUCTION===\n";
	myNavigator.ConstructNavPlan(mySensorHub);

	// Make sure the nav plan was properly constructed.
	if(!myNavigator.IsConstructed()){
		std::cout << "ERROR: Nav Plan not properly constructed. Exiting program.\n";
		return false;
	}else{
		myNavigator.PopulateMovements(mySensorHub);
	}

	return true;
}

void MainOperations(SensorHub& mySensorHub, Navigator& myNavigator, Guider& myGuider, Controller& myController, TimeModule& tm){

	std::cout << "Running...\n";

	tm.InitProccessCounter("Nav", 0.1);
	tm.InitProccessCounter("Guid", 0.1);
	tm.InitProccessCounter("Ctrl", 0.1);
	tm.InitProccessCounter("Write", 0.01);
	tm.InitProccessCounter("Print", 1.0);

	#ifdef SIM
	tm.InitProccessCounter("Plant", 0.01);
	// Generic testing
	PlantModel::GetVehicle()->heading = -180.0;
	myController.SetMotorLSpeed(1.0);
	myController.SetMotorRSpeed(1.0);
	#endif

	std::ofstream output;
	output.open("out.csv");

	// Main logic loop - I EXPECT TO BE HERE FOR A WHILE
	bool running = true;
	while(running){

		#ifdef SIM
		#ifdef DEBUG
		tm.Run();
		#endif
		if(tm.ProccessUpdate("Plant")){
			PlantModel::Run(tm.GetLastProccessDelta("Plant"));
		}
		#endif

		// Run guidance and Navigation (and pass the controller)
		if(tm.ProccessUpdate("Nav")){
			myNavigator.Run(mySensorHub);
		}

		if(tm.ProccessUpdate("Guid")){
			myGuider.Run(myNavigator);
		}

		// Run controls
		if(tm.ProccessUpdate("Ctrl")){
			myController.Run(myGuider);
		}

		double lon = mySensorHub.GetGPS().GetCurrentGPSCoordinates().lon;
		double lat = mySensorHub.GetGPS().GetCurrentGPSCoordinates().lat;

		// Print info to screen
		if(tm.ProccessUpdate("Print")){
			PlantModel::PrintStatus();
		}

		// Write plant model info to a file
		if(tm.ProccessUpdate("Write")){
			output << tm.GetElapsedTime("BeginMainOpsTime") << ","
				<< lon << ","
				<< lat << ","
				<< PlantModel::GetVehicle()->heading << "\n";
		}

		// Stop sim after a certain amount of time
		#ifdef SIM
		double elapsed = tm.GetElapsedTime("BeginMainOpsTime");
		if(elapsed >= 10.0){
			running = false;
		}
		#endif

		// Determine if we're all done here.
		if(myGuider.IsNavPlanComplete() && myGuider.GetCurrentGuidanceManeuver().done){
			//std::cout << "COMPLETE!\n";
			//running = false;
		}
	}

	std::cout << "Main operations complete.\n";
	output.close();

	return;
}

void CleanupOperations(){

	return;
}

// Ensure that all connected sensors are working properly.
bool TestSensorConnectivity(){


	return true;
}

Navigator InutNavPlanCoordinates(){

	// Create a new nav plan to populate
	Navigator navPlan;

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
		navPlan = InutNavPlanCoordinates();
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

void PrintNavPlanInfo(Navigator& n, SensorHub& sh){

	std::vector<Coordinate> coords = n.GetWaypoints();
	std::vector<Movement> moves = n.GetMovements();
	Coordinate myLoc = sh.GetGPS().GetCurrentGPSCoordinates();

	std::cout << "===CURRENT NAV PLAN===\n";
	std::cout << "Current location of (" + std::to_string(myLoc.lon) + "," + std::to_string(myLoc.lat) + ").\n";

	for(int i = 0; i < (int)coords.size(); i++){
		//std::cout << std::to_string(moves[i].distance) << "\n";
		std::cout << "Proceed on heading " + std::to_string(moves[i].heading) + "deg for " + std::to_string(moves[i].distance) + "u until Waypoint #" + std::to_string(i) + " at (" + std::to_string(coords.at(i).lon) + "," + std::to_string(coords.at(i).lat) + ").\n";
	}

	return;
}
