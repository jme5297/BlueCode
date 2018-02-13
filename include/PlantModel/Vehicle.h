#pragma once
#include <cmath>
#include <vector>
#include <Parser.h>

using namespace sensors;

namespace Plant {

	struct pGPS {
	public:
		bool initialized;
		Coordinate coords;
	};
	struct pCam {
	public:
		bool initialized;
		bool enabled;
	};
	struct pMot {
	public:
		bool initialized;
		double val;
	};

	/*!
	 * Main class that represents the simulated vehicle model.
	 * @todo Allow configuration of this class from a config file, rather than
	 * by modifying the constructor.
	 * @todo Modify which of these parameters are public and make a smarter class.
	 */
	class Vehicle {
	public:
		Vehicle();
		~Vehicle();
		void Initialize();
		void InitializeGPS();

		// Generic vehicle properties
		double width;     ///< Vehicle width from far left to far right
		double length;    ///< Vehicle legnth from far front to far back
		double height;

		double heading;   ///< Current vehicle heading. @todo This really shouldn't be required here.

		/// For determining the type of vehicle.
		VehicleType vehicleType;
		double wheelSpeedN;     ///< Normalized wheel speed from 0.0 to 1.0;
		double wheelSteeringN;  ///< Normalized wheel steering direction from -1.0 to 1.0
								/**
								 * - **-1.0**: Complete left turn
								 * - **1.0**: Complete right turn
								 */
		double maxWheelSteeringAngleDeg; ///< Max allowable angle of steering changes for vehicle wheels

		double maxSpeedMPS; ///< Max allowable speed in meters per second.
							/** \note This applies to both vehicle types. */

		// Sensor types - generic
		pGPS gps;
		pCam cam;
		std::vector<pLas> lasers;

		// Motor control for track vehicles
		pMot motL;
		pMot motR;
	};
}
