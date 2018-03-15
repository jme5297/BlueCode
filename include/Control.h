#pragma once
#include <vector>
#include <Guidance.h>

#ifdef SIM
#include <PlantModel/PlantModel.h>
#endif

/// Motor control and actuator control classes and members
namespace Control{

	using namespace Guidance;
	using namespace sensors;

	/*!
	 * Motor/actuator control routines.
	 * @todo There needs to be normalization constraints on setting speeds and steering.
	 */
	class Controller{
	public:
		Controller();

		void Run(Guider* g, SensorHub* sh);						///< Main execution function for the Controller.
		void SetCurrentVehicleMode(VehicleMode vm);	///< Set the current vehicle mode.
		void SetMaxCameraAttempts(int i) { maxCameraAttempts = i; }

		VehicleMode GetCurrentVehicleMode();		///< Get the currently operating vehicle mode.

		void SetMaxTurnSteering(double d);

		// Controls
		void PayloadDrop(Guider* g, SensorHub* sh);

		// Motor Controls
		void InitializeMotorControl();
		void InitializeSteeringControl();
		void EmergencyShutdown();

	protected:
		// For track system
		double motorLSpeed;
		double motorRSpeed;

		// For wheel system
		double wheelSpeedN;
		double wheelSteeringN;

		// This is now a static variable in Control.cpp.
		// double currentWheelSpeed;

		VehicleMode currentVehicleMode;
		GuidanceManeuver currentGuidanceManeuver;

		double maxTurnSteering;
		int maxCameraAttempts;

	};
}
