#pragma once
#include <vector>
#include <Guidance.h>

#ifdef SIM
#include <PlantModel/PlantModel.h>
#endif

/// Motor control and actuator control classes and members
namespace Control {

	using namespace Guidance;
	using namespace sensors;

	/*!
	 * Motor/actuator control routines.
	 * @todo There needs to be normalization constraints on setting speeds and steering.
	 */
	class Controller {
	public:
		Controller();

		void Run(Guider* g, SensorHub* sh);						///< Main execution function for the Controller.
		void SetMaxCameraAttempts(int i) { maxCameraAttempts = i; }
		void SetMaxTurnSteering(double d);

		// Controls
		void PayloadDrop(Guider* g, SensorHub* sh);

		// Motor Controls
		void InitializeMotorControl();
		void InitializeSteeringControl();
		void EmergencyShutdown();

	protected:

		GuidanceManeuver currentGuidanceManeuver;
		double maxTurnSteering;
		int maxCameraAttempts;

	};
}
