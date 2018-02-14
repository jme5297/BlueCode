#pragma once
#include <sensors/GPS/GPS_generic.h>
#include <sensors/Laser/Laser_generic.h>
#include <sensors/Camera/Camera_generic.h>

/// All sensor-related classes and members
namespace sensors {

	/*!
	 * Main class for handling all communications with sensors. This class contains
	 * references to all of the sensors on-board the vehicle, and all communications
	 * happen with the sensors directly through this class. Class definitions commonly
	 * have their own file, where struct definitions are stored in Parser.h.
	 *
	 * The following sensors communicate through the following:
	 *  - **GPS**: Through the GetGPS() function
	 *  - **Lasers**: Through the GetLasers() function
	 *  - **Camera**: Through the GetCamera() function.
	 */
	class SensorHub {
	public:
		/*!
		 * The constructor handles the constructor for all of the
		 * sensor classes.
		 */
		SensorHub();

		/// Run all of the sensor initialization functions.
		bool InitAllSensors();
		/// Run all of the sensor reset functions.
		bool ResetAllSensors();
		/// Returns a pointer to the GPS class on the Sensor Hub.
		GPS* GetGPS() { return &gps0; }
		/// Returns a pointer to the vector of laser classes in the Sensor Hub.
		std::vector<Laser> GetLasers() { return lasers; }
		/// Returns a pointer to the Camera class in the sensor hub.
		Camera* GetCamera() { return &cam0; }

	protected:
		/// Main GPS class object.
		GPS gps0;
		/// Main vector of Laser class objects.
		std::vector<Laser> lasers;
		/// Main Camera class object.
		Camera cam0;
	};
}
