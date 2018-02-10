#pragma once
#include <iostream>
#include <vector>
#include <sensors/Sensors_generic.h>

#ifdef SIM
#include <PlantModel/PlantModel.h>
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

	};
}
