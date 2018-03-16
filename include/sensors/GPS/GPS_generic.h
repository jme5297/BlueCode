#pragma once
#include <TimeModule.h>
#include <Parser.h>

#ifdef SIM
#include <PlantModel/PlantModel.h>
#endif

namespace sensors {

	using namespace Times;

	// Main class for GPS capabilities
	class GPS {
	public:
		GPS();
		~GPS();
		void Run();
		bool Init();
		bool Reset();
		Coordinate GetCurrentGPSCoordinates();
		double GetGPSGroundCourse();

#ifdef SIM
		void SetGPSUncertainty(double d) { gpsUncertainty = d; }
#endif

	protected:
		Coordinate currentGPSCoordinates;
		Coordinate lastCoordinates;
		double vehicleHeading;
		double gpsUncertainty;
		double latToM = 111050.0;
		double lonToM = 84397.0;
	};
}
