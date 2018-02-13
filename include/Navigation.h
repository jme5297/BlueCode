#pragma once
#include <vector>
#include <algorithm>
#include <cmath>
#include <sensors/SensorHub.h>
#include <TimeModule.h>

/// All navigation-related classes and members
namespace Navigation {

	using namespace Times;
	using namespace sensors;

	/// Struct for storing information about movement from one Coordinate to the next.
	struct Movement{
		double heading; 	//!< Represents the heading from one coordinate to the next.
		double distance; 	//!< Represents the distance from one coordinate to the next.
	};

	/// Struct that contains all information for a complete path through several Coordinates.
	struct NavPlan{
		std::vector<Coordinate> coordinates; 	//!< List of desired/recommended Coordinates.
		std::vector<Movement> movements; 		//!< List of recommended Movements between Coordinates.
	};

	/*!
	 * Navigation routines and Path-Planning.
	 * This is the main class that handles most of the navigation work for the vehicle. Path-planning
	 * routines handle most of the heavy lifting of making Coordinate connections. Vehicle state functions
	 * are populated every step and are passed to the Guider for decision-making.
	 * \warning Must populate and construct Nav Plan before normal execution.
	 * \todo Determine if nav plan construction routines should be moved to the Guider class.
	 */
	class Navigator{
	public:
		Navigator();

		// Utility functions
		bool IsPopulated();			///< Returns if Nav Plan has been populated with coordinates.
		void AddCoordinate(int index, double c1, double c2);	///< Add a coordinate to the Nav Plan's list of coordinates
		void AddCoordinates(std::vector<Coordinate> coords);	//< Add a collection of coordinates to the list of coordinates
		double CalculateTotalNavPlanDistance(std::vector<Coordinate> coords,
															SensorHub& sh);		///< Calculate entire distance of a certain Nav Plan
		double DistanceBetweenCoordinates(Coordinate c1, Coordinate c2);		///< Calculate distane between two coordinates
		Movement CalculateMovement(Coordinate c1, Coordinate c2);				///< Calculate heading & direction between two coordinates

		// Nav Plan construction functions
		void ConstructNavPlan(SensorHub& sh);		///< Determine the most optimal path between Nav Plan coordinates
		void PopulateMovements(SensorHub& sh);		///< Populate the required headings and distances between optimal path

		// Nav Plan Accessor functions
		NavPlan GetNavPlan();						///< Return the active Nav Plan
		std::vector<Coordinate> GetWaypoints();		///< Return the Waypoints of the current Nav Plan
		std::vector<Movement> GetMovements();		///< Return the movements of the current Nav Plan

		// Main execution loop
		void Run(SensorHub& sh); 		///< Main execution loop for the Navigator.

		// Vehicle State functions.
		double GetHeading(); 			///< Return vehicle heading.
		Coordinate GetCoordinates();	///< Return vehicle GPS coordinates.
		bool IsNavPlanComplete();		///< Check if NavPlan is complete.

		/// Return information about the laser sensors.
		std::vector<bool> GetPathClearances(){ return isPathObstructed; }
	protected:

		// Utility functions for nav plan construction
		void SwapCoordinates(Coordinate& a, Coordinate& b);					///< Swap two coordinates in a vector of coordinates
		void PrintCoordinatePermutation(std::vector<Coordinate>& vec);		///< Print a specific permutation
		void GenerateAllCoordinatePermutations(std::vector<Coordinate>& coords, unsigned int nextIndex); 	///< Generate all permutations from a list
		std::vector< std::vector<Coordinate> > allCoordinatePermutations;	///< Vector for storing all permutation combinations
		int totalPermutations;												///< Total amount of permutations

		// Current Nav Plan connected to this Nav Planner class
		NavPlan activeNavPlan;			///< Current/active NavPlan
		double vehicleHeading;			///< Current vehicle heading
		Coordinate lastCoordinates;		///< Last trusted GPS coordinates

		std::vector<bool> isPathObstructed; ///< Do the laser readings show an obstructed path?

		double latToM = 111050.0;
		double lonToM = 84397.0;
	};
}
