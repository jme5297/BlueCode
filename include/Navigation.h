#pragma once
#include <sensors/SensorHub.h>

/// All navigation-related classes and members
namespace Navigation {

	using namespace Times;
	using namespace sensors;

	/*!
	 * Navigation routines and Path-Planning.
	 * This is the main class that handles most of the navigation work for the vehicle. Path-planning
	 * routines handle most of the heavy lifting of making Coordinate connections. Vehicle state functions
	 * are populated every step and are passed to the Guider for decision-making.
	 * \warning Must populate and construct Nav Plan before normal execution.
	 * \todo Determine if nav plan construction routines should be moved to the Guider class.
	 */
	class Navigator {
	public:

		/// Initialize all protected parameters in Navigation.
		void Initialize(SensorHub* sh);

		// Utility functions
		bool IsPopulated();										///< Returns if Nav Plan has been populated with coordinates.
		void AddCoordinate(Coordinate c);	///< Add a coordinate to the Nav Plan's list of coordinates
		void AddCoordinates(std::vector<Coordinate> coords);	///< Replace the current Nav Plan coordinates with a new vector of coordinates.
		double CalculateTotalNavPlanDistance(std::vector<Coordinate> coords);	///< Calculate entire distance of a certain Nav Plan
		double DistanceBetweenCoordinates(Coordinate c1, Coordinate c2);		///< Calculate distane between two coordinates
		Movement CalculateMovement(Coordinate c1, Coordinate c2);				///< Calculate heading & direction between two coordinates

		// Nav Plan construction functions
		void ConstructNavPlan(int);					///< Determine the most optimal path between Nav Plan coordinates
		void PopulateMovements();		///< Populate the required headings and distances between optimal path

		// Nav Plan Accessor functions
		NavPlan GetNavPlan();						///< Return the active Nav Plan
		std::vector<Coordinate> GetWaypoints();		///< Return the Waypoints of the current Nav Plan
		std::vector<Movement> GetMovements();		///< Return the movements of the current Nav Plan

		// Main execution loop
		void Run(SensorHub* sh); 		///< Main execution loop for the Navigator.

		// Vehicle State functions.
		Coordinate GetInitialNavPosition() { return initialPosition; }
		double GetHeading(); 			///< Return vehicle heading.
		Coordinate GetCoordinates();	///< Return vehicle GPS coordinates.
		bool IsNavPlanComplete();		///< Check if NavPlan is complete.
		std::vector<int> GetPathObstructions() { return isPathObstructed; } ///< Return information about the laser sensor readings.
	protected:

		// Utility functions for nav plan construction
		void SwapCoordinates(Coordinate& a, Coordinate& b);					///< Swap two coordinates in a vector of coordinates
		void PrintCoordinatePermutation(std::vector<Coordinate>& vec);		///< Print a specific permutation
		void GenerateAllCoordinatePermutations(std::vector<Coordinate>& coords, unsigned int nextIndex); 	///< Generate all permutations from a list
		std::vector< std::vector<Coordinate> > allCoordinatePermutations;	///< Vector for storing all permutation combinations
		int totalPermutations;												///< Total amount of permutations

		// Current Nav Plan connected to this Nav Planner class
		NavPlan activeNavPlan;				///< Current/active NavPlan
		double vehicleHeading;				///< Current vehicle heading
		Coordinate lastCoordinates;			///< Last trusted GPS coordinates
		Coordinate curPos;
		Coordinate initialPosition;			///< Initial position of the vehicle.

		std::vector<int> isPathObstructed;	///< Do the laser readings show an obstructed path?
		double latToM = 111050.0;
		double lonToM = 84397.0;
	};
}
