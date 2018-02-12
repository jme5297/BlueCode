#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <sensors/Sensors_generic.h>
#include <PlantModel/Vehicle.h>
#include <Control.h>

using namespace Control;
using namespace Plant;
using namespace sensors;

class Parser{

public:
  static void ReadInputs(std::string file);
  static std::vector<Coordinate> GetInputCoordinates(){ return inputCoords; }
  static double GetRefresh_NAV(){ return Refresh_NAV; }
  static double GetRefresh_GUID(){ return Refresh_GUID; }
  static double GetRefresh_CTRL(){ return Refresh_CTRL; }
  static double GetRefresh_OUT(){ return Refresh_OUT; }
  static bool GetOptimize(){ return Optimize; }
  static double GetPayloadDropRadius(){ return PayloadDropRadius; }
  static double GetOffAngleDeviate(){ return OffAngleDeviate; }
  static double GetOffAngleAccepted(){ return OffAngleAccepted; }
  static double GetCalibrationTime(){ return CalibrationTime; }
  static double GetObstacleDivergenceTime(){ return ObstacleDivergenceTime; }
  static double GetMaxTurnSpeed(){ return MaxTurnSpeed; }
  static int GetMaxCameraAttempts(){ return MaxCameraAttempts; }
  static double GetPayloadServoTime(){ return PayloadServoTime; }
  static VehicleMode GetControlMode(){ return ControlMode; }
  static double GetTimeDelta(){ return TimeDelta; }
  static double GetSimDelta(){ return SimDelta; }
  static double GetMaxSpeedMPS(){ return MaxSpeedMPS; }
  static VehicleType GetVehicleTypeSim(){ return VehicleTypeSim; }

protected:
  static double Refresh_NAV;
  static double Refresh_GUID;
  static double Refresh_CTRL;
  static double Refresh_OUT;
  static std::ifstream configFile;
  static std::vector<Coordinate> inputCoords;
  static bool Optimize;
  static double PayloadDropRadius;
  static double OffAngleDeviate;
  static double OffAngleAccepted;
  static double CalibrationTime;
  static double ObstacleDivergenceTime;
  static double MaxTurnSpeed;
  static int MaxCameraAttempts;
  static double PayloadServoTime;
  static VehicleMode ControlMode;
  static double TimeDelta;
  static double SimDelta;
  static double MaxSpeedMPS;
  static VehicleType VehicleTypeSim;
  static double InitialLatitude;
  static double InitialLongitude;
  static double InitialHeading;

};
