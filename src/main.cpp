/**
 * This is at the top of main.
 * \author Jason Everett
 */

#include <PlantModel/PlantModel.h>
#include <Navigation.h>
#include <Guidance.h>
#include <Control.h>
#include <TimeModule.h>
#include <iostream>
#include <fstream>
#include <chrono>
#include <string>

using namespace Navigation;
using namespace Guidance;
using namespace Control;
using namespace Plant;
using namespace std::chrono;
using namespace Times;

// FUNCTION PROTOTYPES
void InutNavPlan(); 	///< Method for inputting nav-plan information
bool ProgramSetup(SensorHub& mySensorHub,
									Navigator& myNavigator,
									Guider& myGuider,
									Controller& myController);
void MainOperations(SensorHub& mySensorHub,
									Navigator& myNavigator,
									Guider& myGuider,
									Controller& myController);
void CleanupOperations();
bool TestSensorConnectivity();
Navigator InutNavPlanCoordinates();
void PrintNavPlanInfo(Navigator& n, SensorHub& sh);

/** Main logic entrance.
 * This is where the main program enters.
 */
int main(int argc, char* argv[]){

	// Create all of the structures that we'll need.
	Navigator myNavigator;
	Guider myGuider;
	SensorHub mySensorHub;
	Controller myController;

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
	MainOperations(mySensorHub, myNavigator, myGuider, myController);

	std::cin.get();
	std::cout << "Press any key to finish...";
	std::cin.get();
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

	// Set the vehicle mode
	myController.SetCurrentVehicleMode(Control::VehicleMode::Wheel);

	#ifdef SIM
	PlantModel::GetVehicle()->vehicleType = VehicleType::Wheel;
	#endif

	return true;
}

void MainOperations(SensorHub& mySensorHub, Navigator& myNavigator, Guider& myGuider, Controller& myController){

	std::cout << "Running...\n";

#ifdef SIM
	PlantModel::Initialize();
#ifdef DEBUG
	TimeModule::SetTimeSimDelta(0.0001);
#endif
#endif

	TimeModule::AddMilestone("BeginMainOpsTime");
	TimeModule::InitProccessCounter("Nav", 0.05);
	TimeModule::InitProccessCounter("Guid", 0.05);
	TimeModule::InitProccessCounter("Ctrl", 0.05);
	TimeModule::InitProccessCounter("Write", 0.01);
	TimeModule::InitProccessCounter("Print", 2.0);

	#ifdef SIM
	TimeModule::InitProccessCounter("Plant", 0.01);
	// Generic testing

	/// @todo This needs to be accounted for by adding a calibration period.
	PlantModel::GetVehicle()->heading = 360.0-45.0;
	// myController.SetMotorLSpeed(1.0);
	// myController.SetMotorRSpeed(1.0);
	#endif

	std::ofstream output;
	output.open("out/data.csv");

	// Main logic loop - I EXPECT TO BE HERE FOR A WHILE
	bool running = true;
	while(running){

		#ifdef SIM
		#ifdef DEBUG
		TimeModule::Run();
		#endif
		if(TimeModule::ProccessUpdate("Plant")){
			PlantModel::Run(TimeModule::GetLastProccessDelta("Plant"));
		}
		#endif

		// Run guidance and Navigation (and pass the controller)
		if(TimeModule::ProccessUpdate("Nav")){
			myNavigator.Run(mySensorHub);
		}

		if(TimeModule::ProccessUpdate("Guid")){
			myGuider.Run(myNavigator);
		}

		// Run controls
		if(TimeModule::ProccessUpdate("Ctrl")){
			myController.Run(myGuider, mySensorHub);
		}

		double lon = mySensorHub.GetGPS().GetCurrentGPSCoordinates().lon;
		double lat = mySensorHub.GetGPS().GetCurrentGPSCoordinates().lat;

		// Print info to screen
		if(TimeModule::ProccessUpdate("Print")){
			PlantModel::PrintStatus();
		}

		// Write plant model info to a file
		if(TimeModule::ProccessUpdate("Write")){
			output << TimeModule::GetElapsedTime("BeginMainOpsTime") << ","
				<< lon << ","
				<< lat << ","
				<< PlantModel::GetVehicle()->heading << "\n";
		}

		// Stop sim after a certain amount of time
		#ifdef SIM
		double elapsed = TimeModule::GetElapsedTime("BeginMainOpsTime");
		if(elapsed >= 3600.0){
			running = false;
		}
		#endif

		// Determine if we're all done here.
		if(myGuider.IsNavPlanComplete()){
			//std::cout << "COMPLETE!\n";
			running = false;
		}
	}

	std::cout << "Main operations complete.\n";
	output.close();

	return;
}

void CleanupOperations(){
	#ifdef SIM
	PlantModel::Cleanup();
	#endif
	return;
}

// Ensure that all connected sensors are working properly.
bool TestSensorConnectivity(){


	return true;
}

Navigator InutNavPlanCoordinates(){

	// Create a new nav plan to populate
	Navigator navigator;

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
		navigator.AddCoordinate(-1, c1, c2);

		index++;

		std::cout << "Add waypoint " + std::to_string(index) + "? (1=y,0=n,-1=redo): ";
		std::cin >> response;
	}

	// If an undo is requested, then get recursive!
	if(response == -1)
	{
		navigator = InutNavPlanCoordinates();
	}

	std::ofstream pts;
	pts.open("out/pts.csv");
	std::vector<Coordinate> coords = navigator.GetWaypoints();
	for(int i = 0; i < (int)coords.size(); i++){
		pts << coords[i].lon << "," << coords[i].lat << "\n";
	}
	pts.close();

	return navigator;
}

void PrintNavPlanInfo(Navigator& n, SensorHub& sh){

	std::vector<Coordinate> coords = n.GetWaypoints();
	std::vector<Movement> moves = n.GetMovements();
	Coordinate myLoc = sh.GetGPS().GetCurrentGPSCoordinates();

	std::cout << "===CURRENT NAV PLAN===\n";
	std::cout << "Current location of (" + std::to_string(myLoc.lon) + "," + std::to_string(myLoc.lat) + ").\n";

	for(int i = 0; i < (int)coords.size(); i++){
		std::cout << "Proceed on heading " + std::to_string(moves[i].heading) + "deg for " + std::to_string(moves[i].distance) + "u until Waypoint #" + std::to_string(i) + " at (" + std::to_string(coords.at(i).lon) + "," + std::to_string(coords.at(i).lat) + ").\n";
	}

	return;
}
