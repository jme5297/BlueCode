#pragma once
#include <iostream>
#include <vector>
#include <Parser.h>

#ifdef SIM
#include <PlantModel/PlantModel.h>
#include <cstdlib>
#include <ctime>
#endif

namespace sensors{

	// Main class for GPS capabilities
	class GPS{
	public:
		GPS();
		~GPS();
		bool Init();
		bool Reset();
		Coordinate GetCurrentGPSCoordinates();
#ifdef SIM
		void SetGPSUncertainty(double d) { gpsUncertainty = d; }
#endif

	protected:
#ifdef SIM
		double gpsUncertainty;
		double latToM = 111050.0;
		double lonToM = 84397.0;
#endif
	};
}
