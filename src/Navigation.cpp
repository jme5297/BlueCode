#include <Navigation.h>

#ifdef SIM
#include <random>
#endif

using namespace sensors;
using namespace Navigation;
using namespace Times;

void Navigator::Initialize(SensorHub* sh)
{
	totalPermutations = 0;
	vehicleHeading = 0.0;
	curPos = sh->GetGPS()->GetCurrentGPSCoordinates();
	lastCoordinates = curPos;
	initialPosition = curPos;
}
/**
 *
 * @param[in]	sh 	- A reference to the Sensor Hub class.
 */
void Navigator::Run(SensorHub* sh)
{
	double PI = 3.14159265;

	// Retrieve laser sensor information->
	isPathObstructed.clear();
	for (int i = 0; i < sh->GetLasers().size(); i++) {
		if (sh->GetLasers().at(i).ReadLaser()) {
			isPathObstructed.push_back(1);
		}
		else {
			isPathObstructed.push_back(0);
		}
	}

	// Run the GPS.
	lastCoordinates = curPos;
#ifdef SIM
	sh->GetGPS()->Run();
#endif
	curPos = sh->GetGPS()->GetCurrentGPSCoordinates();
	vehicleHeading = sh->GetGPS()->GetGPSGroundCourse();

	return;
}
Coordinate Navigator::GetCoordinates()
{
//	std::cout << "curPosLat: " << curPos.lat << "\n";
	return curPos;
}
// Add a coordinate to the vector list of coordinates of the specific active Nav Plan->
// NOTE: This must be done before constructing the Nav Plan->
void Navigator::AddCoordinate(int index, double c1, double c2)
{
	// Create a coordinate from the two incoming values.
	Coordinate coord;
	coord.lon = c1;
	coord.lat = c2;

	// Retrieve the current coordinates in the active nav plan
	std::vector<Coordinate> myCoords = activeNavPlan.coordinates;

	// If index is -1, then add to the end of the list. Otherwise, insert
	// the waypoint at a specific location->
	if (index == -1)
	{
		myCoords.push_back(coord);
	}
	else if (index <= (int)myCoords.size() - 1)
	{
		std::vector<Coordinate>::iterator it = myCoords.begin();
		myCoords.insert(it + index, coord);
	}

	// Update the vector of coordinates to the active Nav Plan
	activeNavPlan.coordinates = myCoords;
	return;
}

// Reorganize Nav Plan for most optimal path.
void Navigator::ConstructNavPlan(int cInd)
{

	// This function populates the allCoordinatePermutations vector
	allCoordinatePermutations.clear();
	std::vector<Coordinate> coordsToGo;

	// All except last (last needs to remain the same for returning to original location)
	// hence size() - 1
	for (int i = cInd; i < activeNavPlan.coordinates.size() - 1; i++) {
		coordsToGo.push_back(activeNavPlan.coordinates[i]);
	}

	GenerateAllCoordinatePermutations(coordsToGo, 0);

	// Add the initial position back to the end.
	for (int i = 0; i < allCoordinatePermutations.size(); i++) {
		allCoordinatePermutations[i].push_back(initialPosition);
	}

	TimeModule::Log("NAV", "Found " + std::to_string(allCoordinatePermutations.size()) + " path options.");

	// Iterate to find the smallest distance from the permutations.
	int shortestDistanceIndex = -1;
	double shortestDistance = 1.0e30;
	for (unsigned int i = 0; i < allCoordinatePermutations.size(); i++)
	{
		double dist = CalculateTotalNavPlanDistance(allCoordinatePermutations[i]);
		if (dist < shortestDistance)
		{
			shortestDistance = dist;
			shortestDistanceIndex = i;
		}
	}

	TimeModule::Log("NAV", "Path " + std::to_string(shortestDistanceIndex) + " is the shortest!");

	// Clear the current plan->
	std::vector<Coordinate> tmpCoords = activeNavPlan.coordinates;
	activeNavPlan.coordinates.clear();
	for (int i = 0; i < cInd; i++)
	{
		activeNavPlan.coordinates.push_back(tmpCoords[i]);
	}
	// Update the nav plan coordinate order
	for (int i = 0; i < allCoordinatePermutations[shortestDistanceIndex].size(); i++)
	{
		activeNavPlan.coordinates.push_back(allCoordinatePermutations[shortestDistanceIndex][i]);
	}

	return;
}

// Populate the movement array, which stores heading and distance information to all waypoints.
// NOTE: ConstructNavPlan() is not required to be run, but is highly recommended.
void Navigator::PopulateMovements(SensorHub* sh)
{

	// Create a generic movement vector, and store the active Nav Plan coordinates.
	std::vector<Movement> moves;
	std::vector<Coordinate> coords = activeNavPlan.coordinates;

	// First movement is from the current location to the first nav plan coordinate.
	moves.push_back(CalculateMovement(curPos, coords[0]));

	// All other movements are from nav plan coordinates to other nav plan coordinates.
	for (unsigned int i = 0; i < coords.size() - 1; i++)
	{
		moves.push_back(CalculateMovement(coords[i], coords[i + 1]));
	}

	// If we have to return to the original location, then include this.
	// moves.push_back(CalculateMovement(coords[coords.size() - 1], initialPosition));

	// Update the total movement vector to the Active Nav Plan->
	activeNavPlan.movements = moves;

	return;
}

// Calculate the heading and distance between two coordinates.
Movement Navigator::CalculateMovement(Coordinate c1, Coordinate c2)
{
	double PI = 3.14159265;

	double dx = c2.lon - c1.lon;
	double dy = c2.lat - c1.lat;
	double z = atan2(dy, dx) * 180.0 / PI;
	double head = 90.0 - z;

	Movement m;
	m.heading = (head < 0.0) ? 360.0 + head : head;
	m.distance = DistanceBetweenCoordinates(c1, c2);;

	return m;
}

// Calculate the total distance from a specific set of coordinates.
double Navigator::CalculateTotalNavPlanDistance(std::vector<Coordinate> coords)
{

	double totalDistance = 0.0;
	totalDistance += DistanceBetweenCoordinates(curPos, coords[0]);

	for (unsigned int i = 0; i < coords.size() - 1; i++)
	{
		totalDistance += DistanceBetweenCoordinates(coords[i], coords[i + 1]);
	}

	// If we have to return to the original location, then include this.
	// totalDistance += DistanceBetweenCoordinates(curPos, initialPosition);

	return totalDistance;
}

// Main permutation function (recursive) for coordinate vector permutations.
//
// Source:
// http://www.cplusplus.com/forum/general/44552/
void Navigator::GenerateAllCoordinatePermutations(std::vector<Coordinate>& coords, unsigned int nextIndex)
{
	if (nextIndex == coords.size())
	{
		totalPermutations++;
		//PrintCoordinatePermutation(coords);
		allCoordinatePermutations.push_back(coords);
		//std::cout << " - " + std::to_string(totalPermutations) + "\n";
		return;
	}

	for (unsigned int i = nextIndex; i < coords.size(); i++)
	{
		SwapCoordinates(coords[i], coords[nextIndex]);
		GenerateAllCoordinatePermutations(coords, nextIndex + 1);
		SwapCoordinates(coords[i], coords[nextIndex]);
	}
}

void Navigator::SwapCoordinates(Coordinate& a, Coordinate& b)
{
	Coordinate x = a;
	a = b;
	b = x;
}

// Not currently being used
void Navigator::PrintCoordinatePermutation(std::vector<Coordinate>& vec)
{
	for (unsigned int i = 0; i < vec.size(); i++)
		std::cout << vec[i].lon << "," << vec[i].lat << "||";
}
void Navigator::AddCoordinates(std::vector<Coordinate> coords)
{
	activeNavPlan.coordinates = coords;
}

double Navigator::DistanceBetweenCoordinates(Coordinate c1, Coordinate c2)
{
	return pow(
		pow((c2.lat - c1.lat)*latToM, 2.0) +
		pow((c2.lon - c1.lon)*lonToM, 2.0)
		, 0.5);
}

std::vector<Coordinate> Navigator::GetWaypoints()
{
	return activeNavPlan.coordinates;
}
std::vector<Movement> Navigator::GetMovements()
{
	return activeNavPlan.movements;
}
bool Navigator::IsPopulated()
{
	return activeNavPlan.coordinates.size();
}
NavPlan Navigator::GetNavPlan()
{
	return activeNavPlan;
}
double Navigator::GetHeading()
{
	return vehicleHeading;
}
