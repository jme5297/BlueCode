#pragma once
#include <vector>

#ifdef SIM
#include <PlantModel/PlantModel.h>
#endif

#include <Guidance.h>
#include <sensors/SensorHub.h>

/// Motor control and actuator control classes and members
namespace Control{

	using namespace Guidance;
	using namespace sensors;

	/*!
	 * Represents which type of vehicle the controller is operating.
	 * \note This should not be confused with VehicleType defined in the Vehicle class. This
	 * represents the control maneuvers, whereas the VehicleType represents the plant model being
	 * run. Obviously, these two modes should always be in sync.
	 */
	enum VehicleMode{
		Track, 	//!< Vehicle moves along two tracks, one on each side of the vehicle.
		Wheel 	//!< Vehicle contains four wheels and has both speed/steering control.
	};

	/*!
	 * Motor/actuator control routines.
	 * @todo There needs to be normalization constraints on setting speeds and steering.
	 */
	class Controller{
	public:
		Controller();

		void SetMotorSpeeds(double speed); 		///< Set the normalized speed (-1.0 to 1.0) of both motors in a track vehicle system.
		void SetMotorLSpeed(double speed);		///< Set the normalized speed (-1.0 to 1.0) of the left motor in a track vehicle system.
		void SetMotorRSpeed(double speed);		///< Set the normalized speed (-1.0 to 1.0) of the right motor in a track vehicle system.
		double GetMotorLSpeed();				///< Get the normalized speed (-1.0 to 1.0) of the left motor in a track vehicle system.
		double GetMotorRSpeed();				///< Get the normalized speed (-1.0 to 1.0) of the right motor in a track vehicle system.

		void SetWheelSpeed(double speed);		///< Set the normalized driving speed for a wheel vehicle system.
		void SetWheelSteering(double steering); ///< Set the normalized (-1.0 to 1.0) steering direction for a wheel vehicle system.
		double GetWheelSpeed();					///< Get the normalized driving speed (0.0 to 1.0) for a wheel vehicle system.
		double GetWheelSteering();				///< Get the normalized steering direction (-1.0 to 1.0) for a wheel vehicle system.

		void Run(Guider& g, SensorHub& sh);						///< Main execution function for the Controller.
		void SetCurrentVehicleMode(VehicleMode vm);	///< Set the current vehicle mode.
		VehicleMode GetCurrentVehicleMode();		///< Get the currently operating vehicle mode.

		// Controls
		void PayloadDrop(Guider& g, SensorHub& sh);

	protected:
		// For track system
		double motorLSpeed;
		double motorRSpeed;

		// For wheel system
		double wheelSpeedN;
		double wheelSteeringN;

		VehicleMode currentVehicleMode;
		GuidanceManeuver currentGuidanceManeuver;

	};
}
