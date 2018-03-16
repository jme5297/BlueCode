#pragma once
#include <cmath>
#include <chrono>
#include <iostream>
#include <vector>
#include <TimeModule.h>
#include <PlantModel/Vehicle.h>

#ifdef USE_IRRLICHT
#	if defined(__APPLE__) || defined(MACOSX)
#		include <irrlicht.h>
#		include <OpenGL/OpenGL.h>
#	else
#		ifdef _MSC_VER
# 		pragma comment(linker, "/SUBSYSTEM:CONSOLE")
#		endif
#		include "irrlicht.h"
#	endif
#endif

/// Physics and graphics plant model classes & functions.
namespace Plant {

	using namespace sensors;

#ifdef USE_IRRLICHT
	using namespace irr;
	using namespace core;
	using namespace scene;
	using namespace video;
	using namespace io;
	using namespace gui;

	/*!
	 * Main class for handling the plant model. Most of these members are static because
	 * only one plant model should be running at a given time. This class also handles updating
	 * the Irrlicht engine. Note: The plant model is only compiled into the code if the SIM flag
	 * is defined in the CMake generation.
	 */
	class PlantModel : public IEventReceiver{
#else
	class PlantModel {
#endif
	public:

		/// Used to initialize the plant model and the Irrlicht engine.
		void Initialize();

		#ifdef USE_IRRLICHT
		static void DrawPayloadLocations(std::vector<Coordinate> coords, double pldist);
		static void DrawObstacles(std::vector<Obstacle> obs);
		static void UpdateEngine();         ///< Updates the Irrlicht 3d engine.
		static void SendGPSData(Coordinate, double);	///< Draws GPS information
		static vector3df getPositionOnSphere(f32 angleH, f32 angleV, f32 radius);
		virtual bool OnEvent(const SEvent &event);
		/// Updates an image displayed in the upper left of the Irrlicht window.
		static void UpdateImage(std::string str);
		#endif

		/// Shuts down the plant model and the Irrlicht device that was created.
		static void Cleanup();

		/**
		 * \todo The plant model has not yet been checked to validate backward steering speeds
		 * for both wheel vehicles and track vehicles. Ideally, track vehicles should be able to
		 * pivot around the centroid. This capability should be built in.
		 */
		static void Run(double dt);         ///< Main execution function for the plant model.

		static Coordinate GetGPSCoords();   ///< Get the vehicle's current GPS coordinates.
		static Vehicle * GetVehicle();      ///< Returns a pointer to the main vehicle object in the simulation.
		static double GetElapsedSeconds();  ///< Get elapsed seconds since the plant model was initialized.
		static std::chrono::duration<double> GetSimDuration();  ///< Return a std::chrono::duration of total sim running time.
		static void PrintStatus();          ///< Print some information about the plant model to the output stream.

		static double LatToZ(double d);
		static double LonToX(double d);
		static double XToLon(double d);
		static double ZToLat(double d);

	protected:
		/// Stores the time of intitialization of the plant model.
		static std::chrono::time_point<std::chrono::system_clock> initTime;

		/// The main vehicle that is running in the plant model.
		static Vehicle veh;

		// The initial latitude/longitude will be the point 0,0.
		static double initLat;
		static double initLon;

		static double latToM;
		static double lonToM;
	};
}
