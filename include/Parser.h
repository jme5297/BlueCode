#pragma once
#include <iomanip>
#include <chrono>
#include <vector>
#include <string>
#include <tuple>
#include <iostream>
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
#include <fcntl.h>			//Used for UART
#include <termios.h>		//Used for UART
#include <unistd.h>

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
using namespace std;

class Parser {

public:
	static void ReadInputs(std::string file);
	static std::vector<Coordinate> GetInputCoordinates() { return inputCoords; }
	static double GetRefresh_GNC() { return Refresh_GNC; }
	static double GetRefresh_OUT() { return Refresh_OUT; }
	static double GetRefresh_Gains(){ return Refresh_Gains; }
	static double GetRefresh_GPS() { return Refresh_GPS; }
	static int GetOptimize() { return Optimize; }
	static int GetReOptimize(){ return ReOptimize; }
	static bool GetWriteToLogFile() { return WriteToLogFile; }
	static double GetPayloadDropRadius() { return PayloadDropRadius; }
	static double GetOffAngleDeviate() { return OffAngleDeviate; }

	// Dynamics
	static double GetTurnFactorDPS() { return TurnFactorDPS; }
	static double GetMaxSpeedMPS() { return MaxSpeedMPS; }
	static double GetTurnSpeedFactor(){ return TurnSpeedFactor; }
	static double GetStraightSpeedFactor(){ return StraightSpeedFactor; }
	static double GetMaxAllowableThrottleGain(){ return MaxAllowableThrottleGain; }
	static double GetSpeedSensitivityFactor(){ return SpeedSensitivityFactor; }
	static double GetBackMultipier(){ return BackMultiplier; }
	static double GetBreakFactor(){ return BreakFactor; }

	static double GetCalibrationTime() { return CalibrationTime; }
	static int GetReCalibrate(){ return ReCalibrate; }
	static double GetMinimumMaintainTime() { return MinimumMaintainTime; }
	static double GetObstacleDivergenceAngle() { return ObstacleDivergenceAngle; }
	static double GetObstacleDivergenceTime() { return ObstacleDivergenceTime; }
	static double GetMaxTurnSteering() { return MaxTurnSteering; }
	static int GetMaxCameraAttempts() { return MaxCameraAttempts; }
	static double GetPayloadServoTime() { return PayloadServoTime; }
	static double GetDC_ESC_Fwd(){ return DC_ESC_Fwd; }
	static double GetDC_ESC_Zero(){ return DC_ESC_Zero; }
	static double GetDC_ESC_Back(){ return DC_ESC_Back; }
	static double GetDC_Steer_Left() { return DC_Steer_Left; }
	static double GetDC_Steer_Straight() { return DC_Steer_Straight; }
	static double GetDC_Steer_Right() { return DC_Steer_Right; }
	static double GetDC_Payload_Start() { return DC_Payload_Start; }
	static double GetDC_Payload_Delta() { return DC_Payload_Delta; }
	static double GetPRU_Sample_Rate() { return PRU_Sample_Rate; }
	static double GetPRU_ESC_Delay() { return PRU_ESC_Delay; }
	static double GetPRU_Steer_Delay() { return PRU_Steer_Delay; }
	static int GetCam_Width(){ return Cam_Width; }
	static int GetCam_Height(){ return Cam_Height; }
	static int GetLaser_Left() { return Laser_Left; }
	static int GetLaser_Right() { return Laser_Right; }
	static int GetTemp_Disable_Laser(){ return Temp_Disable_Laser; }
	static int GetGPIO_Steer() { return GPIO_Steer; }
	static int GetGPIO_Payload() { return GPIO_Payload; }

	// Simulation
	static double GetTimeDelta() { return TimeDelta; }
	static double GetSimDelta() { return SimDelta; }
	static double GetTerrainRoughness(){ return TerrainRoughness; }
	static double GetHillFactor(){ return HillFactor; }
	static double GetGPSUncertainty() { return GPSUncertainty; }
	static double GetGPSHeadingUncertainty() { return GPSHeadingUncertainty; }
	static double GetGPSVelocityUncertainty(){ return GPSVelocityUncertainty; }
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
	static double Refresh_GNC;
	static double Refresh_OUT;
	static double Refresh_Gains;
	static double Refresh_GPS;
	static std::ifstream configFile;
	static std::vector<Coordinate> inputCoords;
	static int Optimize;
	static int ReOptimize;
	static bool WriteToLogFile;
	static double PayloadDropRadius;
	static double OffAngleDeviate;

	// Dynamics
	static double MaxSpeedMPS;
	static double TurnFactorDPS;
	static double TurnSpeedFactor;
	static double StraightSpeedFactor;
	static double MaxAllowableThrottleGain;
	static double SpeedSensitivityFactor;
	static double BackMultiplier;
	static double BreakFactor;


	static double CalibrationTime;
	static int ReCalibrate;
	static double MinimumMaintainTime;
	static double ObstacleDivergenceAngle;
	static double ObstacleDivergenceTime;
	static double MaxTurnSteering;
	static int MaxCameraAttempts;
	static double PayloadServoTime;
	static double DC_ESC_Fwd;
	static double DC_ESC_Zero;
	static double DC_ESC_Back;
	static double DC_Steer_Left;
	static double DC_Steer_Straight;
	static double DC_Steer_Right;
	static double DC_Payload_Start;
	static double DC_Payload_Delta;
	static double PRU_Sample_Rate;
	static double PRU_ESC_Delay;
	static double PRU_Steer_Delay;
	static int Cam_Width;
	static int Cam_Height;
	static int Laser_Left;
	static int Laser_Right;
	static int Temp_Disable_Laser;
	static int GPIO_Steer;
	static int GPIO_Payload;

	// Simulation
	static double TimeDelta;
	static double SimDelta;
	static double TerrainRoughness;
	static double HillFactor;
	static double GPSUncertainty;
	static double GPSHeadingUncertainty;
	static double GPSVelocityUncertainty;
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
