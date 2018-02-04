#pragma once
#include <preprocdef.h>
#include <vector>
#include <PlantModel/PlantModel.h>
// #include <sensors/Sensors_generic.h>
// #include <Navigation.h>
// #include <Guidance.h>

namespace Control{

	// using namespace sensors;
	// using namespace Navigation;
	// using namespace Guidance;

	enum VehicleMode{
		Track,
		Wheel
	};

	class Controller{
	public:
		Controller();
		~Controller();

		// These assume track mode
		void SetMotorSpeeds(double speed);
		void SetMotorLSpeed(double speed);
		void SetMotorRSpeed(double speed);
		double GetMotorLSpeed();
		double GetMotorRSpeed();

		// These assume wheel mode
		void SetWheelSpeed(double speed);
		void SetWheelSteering(double steering);
		double GetWheelSpeed();
		double GetWheelSteering();

		// Main execution command
		void Run();

		VehicleMode currentVehicleMode;

		// Controls
		void PayloadDrop();

	protected:
		// For track system
		double motorLSpeed;
		double motorRSpeed;

		// For wheel system
		double wheelSpeed;
		double wheelSteering;

	};
}
