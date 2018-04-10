#pragma once

#ifdef SIM
#include <PlantModel/PlantModel.h>
#endif

#include <TimeModule.h>
#include <Parser.h>

namespace sensors {


	// Main class for handling camera capabilities
	class Camera {
	public:
		Camera();
		~Camera();
		bool Init();
		bool Reset();
		bool Enable();
		bool Disable();
		bool TakeImage(int a);


	};

}
