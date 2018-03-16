#pragma once
#include <iostream>
#include <vector>
#include <iomanip>
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <thread>
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#ifdef TEST_PWM
#include <unistd.h>
#endif

namespace Control {
	/*!
	* Represents which type of vehicle the controller is operating.
	* \note This should not be confused with VehicleType defined in the Vehicle class. This
	* represents the control maneuvers, whereas the VehicleType represents the plant model being
	* run. Obviously, these two modes should always be in sync.
	*/
	enum VehicleMode {
		Track, 	//!< Vehicle moves along two tracks, one on each side of the vehicle.
		Wheel 	//!< Vehicle contains four wheels and has both speed/steering control.
	};
}

namespace sensors {
	struct Coordinate {
	public:
		double lon;
		double lat;
	};
}

namespace Navigation {

	using namespace sensors;

	/// Struct for storing information about movement from one Coordinate to the next.
	struct Movement {
		double heading; 	//!< Represents the heading from one coordinate to the next.
		double distance; 	//!< Represents the distance from one coordinate to the next.
	};
	/// Struct that contains all information for a complete path through several Coordinates.
	struct NavPlan {
		std::vector<Coordinate> coordinates; 	//!< List of desired/recommended Coordinates.
		std::vector<Movement> movements; 		//!< List of recommended Movements between Coordinates.
	};
}

namespace Plant {
	enum VehicleType {
		Track,
		Wheel
	};
	struct pLas {
	public:
		double relativeForward;
		double relativeUpward;
		double relativeRight;
		double direction;
		double detectionDistance;
		bool initialized;
		bool val;
	};
	struct Obstacle {
		double lon;
		double lat;
		double width;
		double length;
		double height;
		double yRotation;
	};
}

using namespace Plant;
using namespace sensors;
using namespace Control;

class Parser {

public:
	static void ReadInputs(std::string file);
	static std::vector<Coordinate> GetInputCoordinates() { return inputCoords; }
	static double GetRefresh_NAV() { return Refresh_NAV; }
	static double GetRefresh_GUID() { return Refresh_GUID; }
	static double GetRefresh_CTRL() { return Refresh_CTRL; }
	static double GetRefresh_OUT() { return Refresh_OUT; }
	static double GetRefresh_GPS() { return Refresh_GPS; }
	static bool GetOptimize() { return Optimize; }
	static double GetPayloadDropRadius() { return PayloadDropRadius; }
	static double GetOffAngleDeviate() { return OffAngleDeviate; }
	static double GetOffAngleAccepted() { return OffAngleAccepted; }
	static double GetTurnFactorDPS() { return TurnFactorDPS; }
	static double GetAccFactor() { return AccFactor; }
	static double GetAccFactorObs() { return AccFactorObs; }
	static double GetCalibrationTime() { return CalibrationTime; }
	static double GetMinimumMaintainTime() { return MinimumMaintainTime; }
	static double GetObstacleDivergenceAngle() { return ObstacleDivergenceAngle; }
	static double GetObstacleDivergenceTime() { return ObstacleDivergenceTime; }
	static double GetMaxTurnSteering() { return MaxTurnSteering; }
	static int GetMaxCameraAttempts() { return MaxCameraAttempts; }
	static double GetPayloadServoTime() { return PayloadServoTime; }
	static VehicleMode GetControlMode() { return ControlMode; }
	static int GetLaser_Left() { return Laser_Left; }
	static int GetLaser_Right() { return Laser_Right; }

	static double GetTimeDelta() { return TimeDelta; }
	static double GetSimDelta() { return SimDelta; }
	static double GetGPSUncertainty() { return GPSUncertainty; }
	static double GetGPSHeadingUncertainty() { return GPSHeadingUncertainty; }
	static double GetMaxSpeedMPS() { return MaxSpeedMPS; }
	static VehicleType GetVehicleTypeSim() { return VehicleTypeSim; }
	static double GetMaxWheelAngleDegrees() { return MaxWheelAngleDegrees; }
	static double GetVehicleWidth() { return VehicleWidth; }
	static double GetVehicleHeight() { return VehicleHeight; }
	static double GetVehicleLength() { return VehicleLength; }
	static double GetInitialLongitude() { return InitialLongitude; }
	static double GetInitialLatitude() { return InitialLatitude; }
	static double GetInitialHeading() { return InitialHeading; }
	static std::vector<Obstacle> GetObstacles() { return Obstacles; }
	static std::vector<pLas> GetPLasers() { return Lasers; }

protected:
	static double Refresh_NAV;
	static double Refresh_GUID;
	static double Refresh_CTRL;
	static double Refresh_OUT;
	static double Refresh_GPS;
	static std::ifstream configFile;
	static std::vector<Coordinate> inputCoords;
	static bool Optimize;
	static double PayloadDropRadius;
	static double OffAngleDeviate;
	static double OffAngleAccepted;
	static double TurnFactorDPS;
	static double AccFactor;
	static double AccFactorObs;
	static double CalibrationTime;
	static double MinimumMaintainTime;
	static double ObstacleDivergenceAngle;
	static double ObstacleDivergenceTime;
	static double MaxTurnSteering;
	static int MaxCameraAttempts;
	static double PayloadServoTime;
	static VehicleMode ControlMode;
	static int Laser_Left;
	static int Laser_Right;

	static double TimeDelta;
	static double SimDelta;
	static double GPSUncertainty;
	static double GPSHeadingUncertainty;
	static double MaxSpeedMPS;
	static VehicleType VehicleTypeSim;
	static double MaxWheelAngleDegrees;
	static double VehicleWidth;
	static double VehicleHeight;
	static double VehicleLength;
	static double InitialLatitude;
	static double InitialLongitude;
	static double InitialHeading;
	static std::vector<Obstacle> Obstacles;
	static std::vector<Plant::pLas> Lasers;

};
