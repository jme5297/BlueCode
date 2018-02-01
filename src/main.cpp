#include <PathPlanning.h>
#include <iostream>
using namespace PathPlanning;

// FUNCTION PROTOTYPES
void InputNavPlan();
bool ProgramSetup(NavPlanner& myNavPlan);
void MainOperations(NavPlanner& myNavPlan);
void CleanupOperations();
bool TestSensorConnectivity();
NavPlanner InputNavPlanCoordinates();
void PrintNavPlanInfo(NavPlanner np);

// MAIN LOGIC ENTRANCE
int main(){

	// Create a generic nav plan
	NavPlanner myNavPlan;

	// ProgramSetup handles constructing the nav plan, and ensuring
	// that all sensors are connected. This will return true if setup has finished correctly.
	bool setup = ProgramSetup(myNavPlan);

	// If failure in setup at any given time, then program cannot continue.
	if(!setup){
		std::cout << "Error setting up program. Exiting program.\n";
		return 0;
	}else{
		std::cout << "Nav Plan constructed successfully!\n\n";
		PrintNavPlanInfo(myNavPlan);
	}

	MainOperations(myNavPlan);
	CleanupOperations();

	return 0;
}

bool ProgramSetup(NavPlanner& myNavPlan)
{
	std::cout << "\nHello!\n\n" << std::endl;

	// Ensure all sensors are connected. If not, exit program.
	std::cout << "Testing sensor connection... \n";
	if(!TestSensorConnectivity()){
		std::cout << "ERROR: Not all sensors have established connectivity. Exiting program.\n";
		return false;
	}else{
		std::cout << "Connection with sensors established!\n";
	}

	// Construct our navigation plan once we know sensors are connected.
	std::cout << "\n===WAYPOINT INPUT===\n";
	myNavPlan = InputNavPlanCoordinates();

	// Make sure that there was at-least one nav plan coordinate added.
	if(!myNavPlan.IsPopulated()){
		std::cout << "ERROR: Nav Plan not properly populated. Exiting program.\n";
		return false;
	}

	// Construct the nav plan based on the waypoints provided.
	std::cout << "\n===NAV PLAN CONSTRUCTION===\n";
	myNavPlan.ConstructNavPlan();

	// Make sure the nav plan was properly constructed.
	if(!myNavPlan.IsConstructed()){
		std::cout << "ERROR: Nav Plan not properly constructed. Exiting program.\n";
		return false;
	}else{
		myNavPlan.PopulateMovements();
	}

	return true;
}

void MainOperations(NavPlanner& myNavPlan){

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

	/*
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
	*/

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

void PrintNavPlanInfo(NavPlanner np){

	std::vector<Coordinate> coords = np.GetWaypoints();
	std::vector<Movement> moves = np.GetMovements();
	Coordinate myLoc = sensors::GPS::GetCurrentGPSCoordinates();

	std::cout << "===CURRENT NAV PLAN===\n";
	std::cout << "Current location of (" + std::to_string(myLoc.lon) + "," + std::to_string(myLoc.lat) + ").\n";

	for(int i = 0; i < (int)coords.size(); i++){
		//std::cout << std::to_string(moves[i].distance) << "\n";
		std::cout << "Proceed on heading " + std::to_string(moves[i].heading) + "deg for " + std::to_string(moves[i].distance) + "u until Waypoint #" + std::to_string(i) + " at (" + std::to_string(coords.at(i).lon) + "," + std::to_string(coords.at(i).lat) + ").\n";
	}

	return;
}
