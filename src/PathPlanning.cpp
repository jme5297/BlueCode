#define PI 3.14159265

#include <PathPlanning.h>
#include <iostream>
using namespace PathPlanning;

NavPlanner::NavPlanner()
{
	isConstructed = false;
	totalPermutations = 0;
}

NavPlanner::~NavPlanner()
{

}

void NavPlanner::AddWaypoint(int index, double c1, double c2)
{
	// Construct coordinate
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

	// Update the coordinates
	activeNavPlan.coordinates = myCoords;
	return;
}

// Reorganize nav plan and construct movement headings for
// the nav plan executed in the shortest amount of time.
void NavPlanner::ConstructNavPlan()
{
	// This function populates the allCoordinatePermutations vector
	GenerateAllCoordinatePermutations(activeNavPlan.coordinates, 0);

	std::cout << std::to_string(totalPermutations) + " possible permutations.\n";

	// Iterate to find the smallest distance from the permutations.
	int shortestDistanceIndex = -1;
	double shortestDistance = 1.0e30;
	for(unsigned int i = 0; i < allCoordinatePermutations.size(); i++)
	{
		double dist = CalculateTotalNavPlanDistance(allCoordinatePermutations[i]);
		if(dist < shortestDistance){
			shortestDistance = dist;
			shortestDistanceIndex = i;
			// std::cout << "Permutation " + std::to_string(i) + " is the shortest so far at " + std::to_string(shortestDistance) + ".\n";
		}
	}

	std::cout << "Permutation " + std::to_string(shortestDistanceIndex) + " is the shortest!\n";

	// Update the nav plan coordinate order
	activeNavPlan.coordinates = allCoordinatePermutations[shortestDistanceIndex];

	// Calculate the distances and headings required.
	ConstructMovements();

	isConstructed = true;
	return;
}

void NavPlanner::ConstructMovements(){

	std::vector<Movement> moves;
	std::vector<Coordinate> coords = activeNavPlan.coordinates;

	// First movement is from the current location to the first nav plan coordinate.
	Coordinate myLoc = sensors::GPS::GetCurrentGPSCoordinates();
	moves.push_back(CalculateMovement(myLoc, coords[0]));

	// All other movements are from nav plan coordinates to other nav plan coordinates.
	for(unsigned int i = 0; i < activeNavPlan.coordinates.size() - 1; i++){
		moves.push_back(CalculateMovement(coords[i], coords[i+1]));
	}

	activeNavPlan.movements = moves;

	return;
}

Movement NavPlanner::CalculateMovement(Coordinate c1, Coordinate c2){

	Movement m;

	double dx = c2.lon - c1.lon;
	double dy = c2.lat - c1.lat;
	double z = atan2(dy, dx) * 180.0 / PI;
	double head = 90.0 - z;
	m.heading = (head < 0.0) ? 360.0 + head : head;

	double dist = DistanceBetweenCoordinates(c1, c2);
	m.distance = dist;

	return m;

}

double NavPlanner::CalculateTotalNavPlanDistance(std::vector<Coordinate> coords)
{
	// Remember, you're not starting at the first location!
	Coordinate myLoc = sensors::GPS::GetCurrentGPSCoordinates();

	double totalDistance = 0.0;

	totalDistance += DistanceBetweenCoordinates(myLoc, coords[0]);

	for(unsigned int i = 0; i < coords.size() - 1; i++)
	{
		totalDistance += DistanceBetweenCoordinates(coords[i], coords[i+1]);
	}

	// If we don't have to return to the original location, this is not necessary.
	// totalDistance += DistanceBetweenCoordinates(myLoc, coords[coords.size() - 1]);

	return totalDistance;
}

double NavPlanner::DistanceBetweenCoordinates(Coordinate c1, Coordinate c2)
{
	double distance = pow( pow( c2.lat - c1.lat ,2.0) + pow( c2.lon - c1.lon ,2.0),0.5);
	return distance;
}

void NavPlanner::GenerateAllCoordinatePermutations(std::vector<Coordinate>& coords, unsigned int nextIndex)
{
	if (nextIndex==coords.size()){
		totalPermutations++;
		//PrintCoordinatePermutation(coords);
		allCoordinatePermutations.push_back(coords);
		//std::cout << " - " + std::to_string(totalPermutations) + "\n";
		return;
	}

	for (unsigned int i = nextIndex; i < coords.size(); i++){
		SwapCoordinates(coords[i], coords[nextIndex]);
		GenerateAllCoordinatePermutations(coords, nextIndex+1);
		SwapCoordinates(coords[i], coords[nextIndex]);
	}
}

void NavPlanner::SwapCoordinates(Coordinate& a, Coordinate& b){
	Coordinate x = a;
	a = b;
	b = x;
}

// Not currently being used
void NavPlanner::PrintCoordinatePermutation(std::vector<Coordinate>& vec){
	for (unsigned int i=0; i<vec.size(); i++)
		std::cout << vec[i].lon << "," << vec[i].lat << "||";
}

std::vector<Coordinate> NavPlanner::GetWaypoints(){ return activeNavPlan.coordinates; }
std::vector<Movement> NavPlanner::GetMovements(){ return activeNavPlan.movements; }
bool NavPlanner::IsPopulated(){ return activeNavPlan.coordinates.size(); }
bool NavPlanner::IsConstructed(){ return isConstructed;}
NavPlan NavPlanner::GetNavPlan(){ return activeNavPlan; }
