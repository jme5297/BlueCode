#ifdef SIM
#include <PlantModel/PlantModel.h>
#endif

#include <Parser.h>
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
using namespace std::chrono;
using namespace Times;

#ifdef SIM
using namespace Plant;
PlantModel pm;
#endif

// FUNCTION PROTOTYPES
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

/*!
 * Main logic entrance.
 * This is where the main program enters.
 */
int main(int argc, char* argv[]) {

	// Read main configuration file
	Parser::ReadInputs("../Config.txt");

	// Create all of the structures that we'll need.
	Navigator myNavigator;
	Guider myGuider;
	SensorHub mySensorHub;
	Controller myController;

	// ProgramSetup handles constructing the nav plan, and ensuring
	// that all sensors are connected. This will return true if setup has finished correctly.
	bool setup = ProgramSetup(mySensorHub, myNavigator, myGuider, myController);

	// If failure in setup at any given time, then program cannot continue.
	if (!setup) {
		std::cout << "Error setting up program. Exiting program.\n";
		return 0;
	}
	else {
		std::cout << "Nav Plan constructed successfully!\n\n";
		PrintNavPlanInfo(myNavigator, mySensorHub);
	}

	// Begin main operations now.
	MainOperations(mySensorHub, myNavigator, myGuider, myController);

	std::cout << "Press return to finish...";
	std::cin.get();
	CleanupOperations();

	return 0;
}

bool ProgramSetup(SensorHub& mySensorHub, Navigator& myNavigator, Guider& myGuider, Controller& myController)
{
	std::cout << "\nHello!\n" << std::endl;

	// Ensure all sensors are connected. If not, exit program.
	std::cout << "Initializing sensor connections... \n";
	if (!mySensorHub.InitAllSensors()) {
		std::cout << "ERROR: Not all sensors have initialized. Exiting program.\n";
		return false;
	}
	else {
		std::cout << "Connection with sensors established!\n";
	}

	// Construct our navigation plan once we know sensors are connected.
	myNavigator.AddCoordinates(Parser::GetInputCoordinates());

	// Make sure that there was at-least one nav plan coordinate added.
	if (!myNavigator.IsPopulated()) {
		std::cout << "ERROR: Nav Plan not properly populated. Exiting program.\n";
		return false;
	}

	// Come up with nominal Nav-Plan movement and distance info.
	myNavigator.PopulateMovements(mySensorHub);

	// Other configuration parameters.
	myGuider.SetPayloadDropRadius(Parser::GetPayloadDropRadius());
	myGuider.SetOffAngleDeviate(Parser::GetOffAngleDeviate());
	myGuider.SetOffAngleAccepted(Parser::GetOffAngleAccepted());
	myGuider.SetCalibrationTime(Parser::GetCalibrationTime());
	myGuider.SetObstacleDivergenceTime(Parser::GetObstacleDivergenceTime());
	myGuider.SetPayloadServoTime(Parser::GetPayloadServoTime());
	myGuider.SetTurnFactorDPS(Parser::GetTurnFactorDPS());
	myGuider.SetMaxVehicleSpeed(Parser::GetMaxSpeedMPS());
	myGuider.SetMinimumMaintainTime(Parser::GetMinimumMaintainTime());
	myGuider.SetObstacleDivergenceAngle(Parser::GetObstacleDivergenceAngle());

	myController.SetCurrentVehicleMode(Parser::GetControlMode());
	myController.SetMaxTurnSteering(Parser::GetMaxTurnSteering());
	// myController.SetMaxCameraAttempts(Parser::GetMaxCameraAttempts());

#ifdef SIM
	PlantModel::GetVehicle()->vehicleType = Parser::GetVehicleTypeSim();
#endif

	return true;
}

void MainOperations(SensorHub& mySensorHub, Navigator& myNavigator, Guider& myGuider, Controller& myController) {

	std::cout << "Running...\n";

#ifdef SIM
	pm.Initialize(
		myNavigator.GetNavPlan().coordinates, 
		Parser::GetObstacles(), 
		myGuider.GetPayloadDropRadius());
#ifdef DEBUG
	TimeModule::SetTimeSimDelta(Parser::GetTimeDelta());
#endif
#endif

	TimeModule::AddMilestone("BeginMainOpsTime");
	TimeModule::InitProccessCounter("Nav", Parser::GetRefresh_NAV());
	TimeModule::InitProccessCounter("Guid", Parser::GetRefresh_GUID());
	TimeModule::InitProccessCounter("Ctrl", Parser::GetRefresh_CTRL());
	TimeModule::InitProccessCounter("Write", Parser::GetRefresh_OUT());

#ifdef SIM
	TimeModule::InitProccessCounter("Plant", Parser::GetSimDelta());
	PlantModel::GetVehicle()->heading = Parser::GetInitialHeading();
	PlantModel::GetVehicle()->gps.coords.lat = Parser::GetInitialLatitude();
	PlantModel::GetVehicle()->gps.coords.lon = Parser::GetInitialLongitude();
#endif

	std::ofstream output;
	output.open("out/data.csv");

	// Main logic loop - I EXPECT TO BE HERE FOR A WHILE
	bool running = true;
	while (running) {

#ifdef SIM
#ifdef DEBUG
		TimeModule::Run();
#endif
		if (TimeModule::ProccessUpdate("Plant")) {
			PlantModel::Run(TimeModule::GetLastProccessDelta("Plant"));
		}
#endif

		// Run guidance and Navigation (and pass the controller)
		if (TimeModule::ProccessUpdate("Nav")) {
			myNavigator.Run(mySensorHub);
		}

		if (TimeModule::ProccessUpdate("Guid")) {
			myGuider.Run(myNavigator);
		}

		// Run controls
		if (TimeModule::ProccessUpdate("Ctrl")) {
			myController.Run(myGuider, mySensorHub);
		}

		double lon = mySensorHub.GetGPS().GetCurrentGPSCoordinates().lon;
		double lat = mySensorHub.GetGPS().GetCurrentGPSCoordinates().lat;

		// Print info to screen
		if (TimeModule::ProccessUpdate("Print")) {
			std::cout << "t: " << TimeModule::GetElapsedTime("BeginMainOpsTime") <<
				" --- lat: " << std::to_string(myNavigator.GetCoordinates().lat) << ", lon: " << myNavigator.GetCoordinates().lon << "\n";
		}

		// Write plant model info to a file
		if (TimeModule::ProccessUpdate("Write")) {
			output << TimeModule::GetElapsedTime("BeginMainOpsTime") << ","
				<< lon << ","
				<< lat << ","
				<< myNavigator.GetHeading() << "\n";
		}

		// Stop sim after a certain amount of time
#ifdef SIM
		double elapsed = TimeModule::GetElapsedTime("BeginMainOpsTime");
		if (elapsed >= 3600.0) {
			running = false;
		}
#endif

		// Determine if we're all done here.
		if (myGuider.IsNavPlanComplete()) {
			//std::cout << "COMPLETE!\n";
			running = false;
		}
	}

	std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "]: Main operations complete.\n";
	output.close();

	return;
}

void CleanupOperations() {
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

void PrintNavPlanInfo(Navigator& n, SensorHub& sh) {

	std::vector<Coordinate> coords = n.GetWaypoints();
	std::vector<Movement> moves = n.GetMovements();
	Coordinate myLoc = sh.GetGPS().GetCurrentGPSCoordinates();

	std::cout << "=======|| Current Nav-Plan ||=======\n";
	std::cout << "Current location of (" + std::to_string(myLoc.lon) + "," + std::to_string(myLoc.lat) + ").\n";

	for (int i = 0; i < (int)coords.size(); i++) {
		std::cout << "Proceed on heading " + std::to_string(moves[i].heading) + "deg for " + std::to_string(moves[i].distance) + "u until Waypoint #" + std::to_string(i) + " at (" + std::to_string(coords.at(i).lon) + "," + std::to_string(coords.at(i).lat) + ").\n";
	}

	return;
}
