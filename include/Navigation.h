#pragma once
#include <vector>
#include <algorithm>
#include <cmath>
#include <sensors/SensorHub.h>
// #include <Guidance.h>
// include <Control.h>

namespace Navigation {

	using namespace sensors;
	// using namespace Guidance;
	// using namespace Control;

	// Second part of a Nav Plan. Contains distance and heading information.
	struct Movement{
		double heading;
		double distance;
	};

	// One Nav Plan contains all information for a complete path through several obstacles.
	struct NavPlan{
		std::vector<Coordinate> coordinates;
		std::vector<Movement> movements;
	};

	// Main class for handling Nav Plan construction and operations.
	class Navigator{
	public:
		Navigator();
		~Navigator();

		// Utility functions
		bool IsPopulated();														// Returns if Nav Plan has been populated with coordinates
		bool IsConstructed();													// Returns if Nav Plan has been constructed and movements have been calculated
		void AddCoordinate(int index, double c1, double c2);					// Add a coordinate to the Nav Plan's list of coordinates
		void AddCoordinates(std::vector<Coordinate> coords);					// Add a collection of coordinates to the list of coordinates
		double CalculateTotalNavPlanDistance(std::vector<Coordinate> coords,
															SensorHub& sh);	// Calculate entire distance of a certain Nav Plan
		double DistanceBetweenCoordinates(Coordinate c1, Coordinate c2);		// Calculate distane between two coordinates
		Movement CalculateMovement(Coordinate c1, Coordinate c2);				// Calculate heading & direction between two coordinates

		// Nav Plan construction functions
		void ConstructNavPlan(SensorHub& sh);		// Determine the most optimal path between Nav Plan coordinates
		void PopulateMovements(SensorHub& sh);		// Populate the required headings and distances between optimal path

		// Accessor functions
		NavPlan GetNavPlan();						// Return the active Nav Plan
		std::vector<Coordinate> GetWaypoints();		// Return the Waypoints of the current Nav Plan
		std::vector<Movement> GetMovements();		// Return the movements of the current Nav Plan

		// Main execution loop
		void Run(SensorHub& sh);

		double GetHeading();
		Coordinate GetCoordinates();
		bool IsNavPlanComplete();

	protected:

		// Utility functions for nav plan construction
		void SwapCoordinates(Coordinate& a, Coordinate& b);					// Swap two coordinates in a vector of coordinates
		void PrintCoordinatePermutation(std::vector<Coordinate>& vec);		// Print a specific permutation [DEBUG ONLY]
		void GenerateAllCoordinatePermutations(std::vector<Coordinate>& coords, unsigned int nextIndex); 	// Generate all permutations from a list
		std::vector< std::vector<Coordinate> > allCoordinatePermutations;	// Vector for storing all permutation combinations
		int totalPermutations;												// Total amount of permutations [DEBUG ONLY]

		// For determining if Nav Plan has been properly been constructed
		bool isConstructed;

		// Current Nav Plan connected to this Nav Planner class
		NavPlan activeNavPlan;
		double vehicleHeading;
		Coordinate lastCoordinates;

	};
}
