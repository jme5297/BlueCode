#pragma once
#include <iostream>
#include <vector>
#include <sensors/Sensors_generic.h>
#include <PlantModel/PlantModel.h>

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
