#include <Parser.h>

using namespace Plant;
using namespace sensors;

// General Configuration
double Parser::Refresh_GNC;
double Parser::Refresh_OUT;
double Parser::Refresh_GPS;
std::vector<Coordinate> Parser::inputCoords;
std::ifstream Parser::configFile;
int Parser::Optimize;
bool Parser::WriteToLogFile;
double Parser::PayloadDropRadius;
double Parser::OffAngleDeviate;
double Parser::TurnFactorDPS;
double Parser::MaxSpeedMPS;
double Parser::TurnSpeedFactor;
double Parser::StraightSpeedFactor;
double Parser::BackMultiplier;
double Parser::BreakFactor;
double Parser::CalibrationTime;
int Parser::ReCalibrate;
double Parser::MinimumMaintainTime;
double Parser::ObstacleDivergenceAngle;
double Parser::ObstacleDivergenceTime;
double Parser::PayloadServoTime;
double Parser::DC_ESC_Fwd;
double Parser::DC_ESC_Zero;
double Parser::DC_ESC_Back;
double Parser::DC_Steer_Left;
double Parser::DC_Steer_Straight;
double Parser::DC_Steer_Right;
double Parser::DC_Payload_Start;
double Parser::DC_Payload_Delta;
double Parser::PRU_Sample_Rate;
double Parser::PRU_ESC_Delay;
double Parser::PRU_Steer_Delay;
double Parser::MaxTurnSteering;
int Parser::MaxCameraAttempts;
int Parser::Laser_Left;
int Parser::Laser_Right;
int Parser::Temp_Disable_Laser;
int Parser::GPIO_Steer;
int Parser::GPIO_Payload;
std::vector<pLas> Parser::Lasers;

// Simulation
double Parser::TimeDelta;
double Parser::SimDelta;
double Parser::GPSUncertainty;
double Parser::GPSHeadingUncertainty;
double Parser::MaxWheelAngleDegrees;
double Parser::VehicleWidth;
double Parser::VehicleHeight;
double Parser::VehicleLength;
double Parser::InitialLatitude;
double Parser::InitialLongitude;
double Parser::InitialHeading;
std::vector<Obstacle> Parser::Obstacles;

void Parser::ReadInputs(std::string file)
{
	configFile.open(file);
	std::string s;

	while (getline(configFile, s)) {
		// Erase any spaces
		s.erase(std::remove(s.begin(), s.end(), ' '), s.end());
		// Skip any comments
		if (s[0] == '#' || s.size() == 0) {
			continue;
		}

		//----------------------------------------
		//        General configuration
		//----------------------------------------
		// Coordinate Saver
		if (s == "BEGIN_COORDINATES") {
			getline(configFile, s);
			s.erase(std::remove(s.begin(), s.end(), ' '), s.end());
			while (s != "END_COORDINATES") {
				// Split into two coordinate values.
				std::stringstream ss(s);
				double lat;
				double lon;
				ss >> lon;
				ss.ignore();  // Ignore the comma
				ss >> lat;
				inputCoords.push_back({ lon, lat });
				getline(configFile, s);
				s.erase(std::remove(s.begin(), s.end(), ' '), s.end());
			}
			std::cout << "Continuing...\n";
			continue;
		}
		else if (s.find("Refresh_GNC") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> Refresh_GNC;
			continue;
		}
		else if (s.find("Refresh_OUT") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> Refresh_OUT;
			continue;
		}
		else if (s.find("Refresh_GPS") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> Refresh_GPS;
			continue;
		}
		else if (s.find("Optimize") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> Optimize;
			std::cout << Optimize << "\n\n\n";
			continue;
		}
		else if (s.find("WriteToLogFile") != std::string::npos) {
			std::string a  = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> WriteToLogFile;
			continue;
		}
		else if (s.find("PayloadDropRadius") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> PayloadDropRadius;
			continue;
		}
		else if (s.find("OffAngleDeviate") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> OffAngleDeviate;
			continue;
		}
		else if (s.find("TurnFactorDPS") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> TurnFactorDPS;
			continue;
		}
		else if (s.find("MaxSpeedMPS") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> MaxSpeedMPS;
			continue;
		}
		else if (s.find("TurnSpeedFactor") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> TurnSpeedFactor;
			continue;
		}
		else if (s.find("StraightSpeedFactor") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> StraightSpeedFactor;
			continue;
		}
		else if (s.find("BackMultipier") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> BackMultiplier;
			continue;
		}
		else if (s.find("BreakFactor") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> BreakFactor;
			continue;
		}
		else if (s.find("CalibrationTime") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> CalibrationTime;
			continue;
		}
		else if (s.find("ReCalibrate") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> ReCalibrate;
			continue;
		}
		else if (s.find("MinimumMaintainTime") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> MinimumMaintainTime;
			continue;
		}
		else if (s.find("ObstacleDivergenceAngle") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> ObstacleDivergenceAngle;
			continue;
		}
		else if (s.find("ObstacleDivergenceTime") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> ObstacleDivergenceTime;
			continue;
		}
		else if (s.find("PayloadServoTime") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> PayloadServoTime;
			continue;
		}
		else if (s.find("DC_ESC_Fwd") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> DC_ESC_Fwd;
			continue;
		}
		else if (s.find("DC_ESC_Zero") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> DC_ESC_Zero;
			continue;
		}
		else if (s.find("DC_ESC_Back") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> DC_ESC_Back;
			continue;
		}
		else if (s.find("DC_Steer_Left") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> DC_Steer_Left;
			continue;
		}
		else if (s.find("DC_Steer_Straight") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> DC_Steer_Straight;
			continue;
		}
		else if (s.find("DC_Steer_Right") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> DC_Steer_Right;
			continue;
		}
		else if (s.find("DC_Payload_Start") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> DC_Payload_Start;
			continue;
		}
		else if (s.find("DC_Payload_Delta") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> DC_Payload_Delta;
			continue;
		}
		else if (s.find("PRU_Sample_Rate") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> PRU_Sample_Rate;
			continue;
		}
		else if (s.find("PRU_ESC_Delay") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> PRU_ESC_Delay;
			continue;
		}
		else if (s.find("PRU_Steer_Delay") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> PRU_Steer_Delay;
			continue;
		}
		else if (s.find("MaxTurnSteering") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> MaxTurnSteering;
			continue;
		}
		else if (s.find("MaxCameraAttempts") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> MaxCameraAttempts;
			continue;
		}
		else if (s.find("Laser_Left") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> Laser_Left;
			continue;
		}
		else if (s.find("Laser_Right") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> Laser_Right;
			continue;
		}
		else if (s.find("Temp_Disable_Laser") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> Temp_Disable_Laser;
			continue;
		}
		else if (s.find("GPIO_Steer") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> GPIO_Steer;
			continue;
		}
		else if (s.find("GPIO_Payload") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> GPIO_Payload;
			continue;
		}
		//----------------------------------------
		//        Simulation configuration
		//----------------------------------------
		else if (s.find("TimeDelta") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> TimeDelta;
			continue;
		}
				else if (s.find("SimDelta") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> SimDelta;
			continue;
		}
		else if (s.find("GPSUncertainty") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> GPSUncertainty;
			continue;
		}
		else if (s.find("GPSHeadingUncertainty") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> GPSHeadingUncertainty;
			continue;
		}
		else if (s.find("MaxWheelAngleDegrees") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> MaxWheelAngleDegrees;
			continue;
		}
		else if (s.find("VehicleWidth") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> VehicleWidth;
			continue;
		}
		else if (s.find("VehicleHeight") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> VehicleHeight;
			continue;
		}
		else if (s.find("VehicleLength") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> VehicleLength;
			continue;
		}
		else if (s.find("InitialLatitude") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> InitialLatitude;
			continue;
		}
		else if (s.find("InitialLongitude") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> InitialLongitude;
			continue;
		}
		else if (s.find("InitialHeading") != std::string::npos) {
			std::string a = s.substr(s.find("=") + 1);
			std::stringstream ss(a);
			ss >> InitialHeading;
			continue;
		}
		else if (s == "BEGIN_OBSTACLES") {
			getline(configFile, s);
			s.erase(std::remove(s.begin(), s.end(), ' '), s.end());
			while (s != "END_OBSTACLES") {
				// Split into two coordinate values.
				std::stringstream ss(s);
				double lat;
				double lon;
				double length;
				double width;
				double height;
				double yRot;
				ss >> lon;
				ss.ignore();  // Ignore the comma
				ss >> lat;
				ss.ignore();  // Ignore the comma
				ss >> length;
				ss.ignore();  // Ignore the comma
				ss >> width;
				ss.ignore();  // Ignore the comma
				ss >> height;
				ss.ignore();  // Ignore the comma
				ss >> yRot;
				Obstacles.push_back({ lon, lat, length, width, height, yRot });
				getline(configFile, s);
				s.erase(std::remove(s.begin(), s.end(), ' '), s.end());
			}
			continue;
		}
		else if (s == "BEGIN_LASERS") {
			getline(configFile, s);
			s.erase(std::remove(s.begin(), s.end(), ' '), s.end());
			while (s != "END_LASERS") {
				// Split into two coordinate values.
				std::stringstream ss(s);
				double relFwd;
				double relUp;
				double relRight;
				double direction;
				double detectionDistance;
				ss >> relFwd;
				ss.ignore();  // Ignore the comma
				ss >> relUp;
				ss.ignore();  // Ignore the comma
				ss >> relRight;
				ss.ignore();  // Ignore the comma
				ss >> direction;
				ss.ignore();
				ss >> detectionDistance;
				Lasers.push_back({ relFwd, relUp, relRight, direction, detectionDistance, true, false });
				getline(configFile, s);
				s.erase(std::remove(s.begin(), s.end(), ' '), s.end());
			}
			continue;
		}
	}
	configFile.close();
	return;
}
