#define PI 3.14159265

#include <Navigation.h>
#include <iostream>

using namespace sensors;
using namespace Navigation;
// using namespace Guidance;

Navigator::Navigator()
{
	isConstructed = false;
	totalPermutations = 0;
}

Navigator::~Navigator()
{

}

/*! Run
 * @param[in]	sh A reference to the Sensor Hub class.
 */
void Navigator::Run(SensorHub& sh)
{

	// Get our current position
	Coordinate curPos = sh.GetGPS().GetCurrentGPSCoordinates();

	// Calculate our heading
	Coordinate c1 = lastCoordinates;
	Coordinate c2 = curPos;

	// Don't know why this is happening
	if((lastCoordinates.lat == curPos.lat)
		&& (lastCoordinates.lon == curPos.lon))
	{
			std::cout << "GPS REPEAT" << std::endl;
		return;
	}

	double dx = c2.lon - c1.lon;
	double dy = c2.lat - c1.lat;
	double z = atan2(dy, dx) * 180.0 / PI;
	double head = 90.0 - z;
	head = (head < 0.0) ? 360.0 + head : head;
	lastCoordinates = curPos;
	vehicleHeading = head;

	return;
}
Coordinate Navigator::GetCoordinates()
{
	return lastCoordinates;
}
// Add a coordinate to the vector list of coordinates of the specific active Nav Plan.
// NOTE: This must be done before constructing the Nav Plan.
void Navigator::AddCoordinate(int index, double c1, double c2)
{
	// Create a coordinate from the two incoming values.
	Coordinate coord;
	coord.lon = c1;
	coord.lat = c2;

	// Retrieve the current coordinates in the active nav plan
	std::vector<Coordinate> myCoords = activeNavPlan.coordinates;

	// If index is -1, then add to the end of the list. Otherwise, insert
	// the waypoint at a specific location.
	if(index == -1)
	{
		myCoords.push_back(coord);
	}else if(index <= (int)myCoords.size() - 1)
	{
		std::vector<Coordinate>::iterator it = myCoords.begin();
		myCoords.insert(it+index, coord);
	}

	// Update the vector of coordinates to the active Nav Plan
	activeNavPlan.coordinates = myCoords;
	return;
}

// Reorganize Nav Plan for most optimal path.
void Navigator::ConstructNavPlan(SensorHub& sh)
{
	// This function populates the allCoordinatePermutations vector
	GenerateAllCoordinatePermutations(activeNavPlan.coordinates, 0);

	std::cout << std::to_string(totalPermutations) + " possible permutations.\n";

	// Iterate to find the smallest distance from the permutations.
	int shortestDistanceIndex = -1;
	double shortestDistance = 1.0e30;
	for(unsigned int i = 0; i < allCoordinatePermutations.size(); i++)
	{
		double dist = CalculateTotalNavPlanDistance(allCoordinatePermutations[i], sh);
		if(dist < shortestDistance)
	{
			shortestDistance = dist;
			shortestDistanceIndex = i;
			std::cout << "Permutation " + std::to_string(i) + " is the shortest so far at " + std::to_string(shortestDistance) + ".\n";
		}
	}

	std::cout << "Permutation " + std::to_string(shortestDistanceIndex) + " is the shortest!\n";

	// Update the nav plan coordinate order
	activeNavPlan.coordinates = allCoordinatePermutations[shortestDistanceIndex];

	isConstructed = true;
	return;
}

// Populate the movement array, which stores heading and distance information to all waypoints.
// NOTE: ConstructNavPlan() is not required to be run, but is highly recommended.
void Navigator::PopulateMovements(SensorHub& sh)
{

	// Create a generic movement vector, and store the active Nav Plan coordinates.
	std::vector<Movement> moves;
	std::vector<Coordinate> coords = activeNavPlan.coordinates;

	// First movement is from the current location to the first nav plan coordinate.
	Coordinate myLoc = sh.GetGPS().GetCurrentGPSCoordinates();
	moves.push_back(CalculateMovement(myLoc, coords[0]));

	// All other movements are from nav plan coordinates to other nav plan coordinates.
	for(unsigned int i = 0; i < coords.size() - 1; i++)
	{
		moves.push_back(CalculateMovement(coords[i], coords[i+1]));
	}

	// If we have to return to the original location, then include this.
	// moves.push_back(CalculateMovement(coords[coords.size() - 1], myLoc));

	// Update the total movement vector to the Active Nav Plan.
	activeNavPlan.movements = moves;

	return;
}

// Calculate the heading and distance between two coordinates.
Movement Navigator::CalculateMovement(Coordinate c1, Coordinate c2)
{

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
double Navigator::CalculateTotalNavPlanDistance(std::vector<Coordinate> coords, SensorHub& sh)
{

	// Remember, you're not starting at the first location!
	Coordinate myLoc = sh.GetGPS().GetCurrentGPSCoordinates();

	double totalDistance = 0.0;
	totalDistance += DistanceBetweenCoordinates(myLoc, coords[0]);

	for(unsigned int i = 0; i < coords.size() - 1; i++)
	{
		totalDistance += DistanceBetweenCoordinates(coords[i], coords[i+1]);
	}

	// If we have to return to the original location, then include this.
	// totalDistance += DistanceBetweenCoordinates(myLoc, coords[coords.size() - 1]);

	return totalDistance;
}

// Main permutation function (recursive) for coordinate vector permutations.
//
// Source:
// http://www.cplusplus.com/forum/general/44552/
void Navigator::GenerateAllCoordinatePermutations(std::vector<Coordinate>& coords, unsigned int nextIndex)
{
	if (nextIndex==coords.size())
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
		GenerateAllCoordinatePermutations(coords, nextIndex+1);
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
	for (unsigned int i=0; i<vec.size(); i++)
		std::cout << vec[i].lon << "," << vec[i].lat << "||";
}

double Navigator::DistanceBetweenCoordinates(Coordinate c1, Coordinate c2)
{ return pow( pow( c2.lat - c1.lat ,2.0) + pow( c2.lon - c1.lon ,2.0),0.5); }
std::vector<Coordinate> Navigator::GetWaypoints()
{ return activeNavPlan.coordinates; }
std::vector<Movement> Navigator::GetMovements()
{ return activeNavPlan.movements; }
bool Navigator::IsPopulated()
{ return activeNavPlan.coordinates.size(); }
bool Navigator::IsConstructed()
{ return isConstructed;}
NavPlan Navigator::GetNavPlan()
{ return activeNavPlan; }
double Navigator::GetHeading()
{ return vehicleHeading; }
