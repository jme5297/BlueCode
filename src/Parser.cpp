#include <Parser.h>

// General Configuration
double Parser::Refresh_NAV;
double Parser::Refresh_GUID;
double Parser::Refresh_CTRL;
double Parser::Refresh_OUT;
std::vector<Coordinate> Parser::inputCoords;
std::ifstream Parser::configFile;
bool Parser::Optimize;
double Parser::PayloadDropRadius;
double Parser::OffAngleDeviate;
double Parser::OffAngleAccepted;
double Parser::CalibrationTime;
double Parser::ObstacleDivergenceTime;
double Parser::PayloadServoTime;
double Parser::MaxTurnSpeed;
int Parser::MaxCameraAttempts;
VehicleMode Parser::ControlMode;

// Simulation
double Parser::TimeDelta;
double Parser::SimDelta;
double Parser::MaxSpeedMPS;
VehicleType Parser::VehicleTypeSim;
double Parser::InitialLatitude;
double Parser::InitialLongitude;
double Parser::InitialHeading;

void Parser::ReadInputs(std::string file)
{
  configFile.open(file);
  std::string s;

  while(getline(configFile, s)){
    // Erase any spaces
    s.erase( std::remove( s.begin(), s.end(), ' ' ), s.end() );
    // Skip any comments
    if(s[0] == '#' || s.size() == 0){
      continue;
    }

    //----------------------------------------
    //        General configuration
    //----------------------------------------
    if(s.find("Refresh_NAV") != std::string::npos){
      std::string a = s.substr(s.find("=")+1);
      std::stringstream ss(a);
      ss >> Refresh_NAV;
      continue;
    }else if(s.find("Refresh_GUID") != std::string::npos){
      std::string a = s.substr(s.find("=")+1);
      std::stringstream ss(a);
      ss >> Refresh_GUID;
      continue;
    }else if(s.find("Refresh_CTRL") != std::string::npos){
      std::string a = s.substr(s.find("=")+1);
      std::stringstream ss(a);
      ss >> Refresh_CTRL;
      continue;
    }else if(s.find("Refresh_OUT") != std::string::npos){
      std::string a = s.substr(s.find("=")+1);
      std::stringstream ss(a);
      ss >> Refresh_OUT;
      continue;
    }else if(s.find("Optimize") != std::string::npos){
      std::string a = s.substr(s.find("=")+1);
      std::stringstream ss(a);
      ss >> Optimize;
      continue;
    }else if(s.find("PayloadDropRadius") != std::string::npos){
      std::string a = s.substr(s.find("=")+1);
      std::stringstream ss(a);
      ss >> PayloadDropRadius;
      continue;
    }else if(s.find("OffAngleDeviate") != std::string::npos){
      std::string a = s.substr(s.find("=")+1);
      std::stringstream ss(a);
      ss >> OffAngleDeviate;
      continue;
    }else if(s.find("OffAngleAccepted") != std::string::npos){
      std::string a = s.substr(s.find("=")+1);
      std::stringstream ss(a);
      ss >> OffAngleAccepted;
      continue;
    }else if(s.find("CalibrationTime") != std::string::npos){
      std::string a = s.substr(s.find("=")+1);
      std::stringstream ss(a);
      ss >> CalibrationTime;
      continue;
    }else if(s.find("ObstacleDivergenceTime") != std::string::npos){
      std::string a = s.substr(s.find("=")+1);
      std::stringstream ss(a);
      ss >> ObstacleDivergenceTime;
      continue;
    }else if(s.find("PayloadServoTime") != std::string::npos){
      std::string a = s.substr(s.find("=")+1);
      std::stringstream ss(a);
      ss >> PayloadServoTime;
      continue;
    }else if(s.find("MaxCameraAttempts") != std::string::npos){
      std::string a = s.substr(s.find("=")+1);
      std::stringstream ss(a);
      ss >> MaxTurnSpeed;
      continue;
    }else if(s.find("MaxCameraAttempts") != std::string::npos){
      std::string a = s.substr(s.find("=")+1);
      std::stringstream ss(a);
      ss >> Optimize;
      continue;
    }else if(s.find("ControlMode") != std::string::npos){
      std::string a = s.substr(s.find("=")+1);
      std::stringstream ss(a);
      std::string b;
      ss >> b;
      if(b == "Wheel"){
        ControlMode = VehicleMode::Wheel;
      }else{
        ControlMode = VehicleMode::Track;
      }
      continue;
    }
    //----------------------------------------
    //        Simulation configuration
    //----------------------------------------
    else if(s.find("TimeDelta") != std::string::npos){
      std::string a = s.substr(s.find("=")+1);
      std::stringstream ss(a);
      ss >> TimeDelta;
      continue;
    }else if(s.find("SimDelta") != std::string::npos){
      std::string a = s.substr(s.find("=")+1);
      std::stringstream ss(a);
      ss >> SimDelta;
      continue;
    }else if(s.find("MaxSpeedMPS") != std::string::npos){
      std::string a = s.substr(s.find("=")+1);
      std::stringstream ss(a);
      ss >> MaxSpeedMPS;
      continue;
    }else if(s.find("VehicleTypeSim") != std::string::npos){
      std::string a = s.substr(s.find("=")+1);
      std::stringstream ss(a);
      std::string b;
      ss >> b;
      if(b == "Wheel"){
        VehicleTypeSim = VehicleType::Wheel;
      }else{
        VehicleTypeSim = VehicleType::Track;
      }
      continue;
    }else if(s.find("InitialLatitude") != std::string::npos){
      std::string a = s.substr(s.find("=")+1);
      std::stringstream ss(a);
      ss >> InitialLatitude;
      continue;
    }
    else if(s.find("InitialLongitude") != std::string::npos){
      std::string a = s.substr(s.find("=")+1);
      std::stringstream ss(a);
      ss >> InitialLongitude;
      continue;
    }
    else if(s.find("InitialHeading") != std::string::npos){
      std::string a = s.substr(s.find("=")+1);
      std::stringstream ss(a);
      ss >> InitialHeading;
      continue;
    }

    // Coordinate Saver
    else if(s == "BEGIN_COORDINATES"){
      getline(configFile, s);
      s.erase( std::remove( s.begin(), s.end(), ' ' ), s.end() );
      while(s != "END_COORDINATES"){
        // Split into two coordinate values.
        std::stringstream ss(s);
        double lat;
        double lon;
        ss >> lon;
        ss.ignore();  // Ignore the comma
        ss >> lat;
        inputCoords.push_back({lon, lat});
        std::cout << lat << "," << lon << "\n";
        getline(configFile, s);
        s.erase( std::remove( s.begin(), s.end(), ' ' ), s.end() );
      }
      continue;
    }

  }
  configFile.close();
  return;
}
