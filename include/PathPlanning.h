#include <vector>
#include <algorithm>
#include <cmath>
#include <sensors/GPS/GPS_generic.h>

namespace PathPlanning {

	using namespace sensors::GPS;

	struct Movement{
		double heading;
		double distance;
	};

	struct NavPlan{
		std::vector<Coordinate> coordinates;
		std::vector<Movement> movements;
	};

	class NavPlanner{
	public:
		NavPlanner();
		~NavPlanner();

		// Path Planning activities
		bool IsPopulated();
		bool IsConstructed();

		// Utility functions
		void AddWaypoint(int index, double c1, double c2);
		void AddWaypoints(std::vector<Coordinate> coords);
		double CalculateTotalNavPlanDistance(std::vector<Coordinate> coords);
		double DistanceBetweenCoordinates(Coordinate c1, Coordinate c2);
		Movement CalculateMovement(Coordinate c1, Coordinate c2);

		// Function for constructing NavPlan from waypoints
		void ConstructNavPlan();
		void ConstructMovements();

		// Accessor functions
		NavPlan GetNavPlan();
		std::vector<Coordinate> GetWaypoints();
		std::vector<Movement> GetMovements();

	protected:

		// Utility functions for nav plan construction
		void SwapCoordinates(Coordinate& a, Coordinate& b);
		void PrintCoordinatePermutation(std::vector<Coordinate>& vec);
		void GenerateAllCoordinatePermutations(std::vector<Coordinate>& coords, unsigned int nextIndex);
		std::vector< std::vector<Coordinate> > allCoordinatePermutations;
		int totalPermutations;

		bool isConstructed;

		NavPlan activeNavPlan;
		std::vector<Coordinate> waypoints;
		std::vector<Movement> movements;
	};

}
