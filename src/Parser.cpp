#include <Parser.h>

std::vector<Coordinate> Parser::inputCoords;
std::ifstream Parser::configFile;
bool Parser::optimizeNavPlan;

void Parser::ReadInputs(string file)
{
  configFile.open(file);
  string s;

  while(getline(configFile, s)){
    // Erase any spaces
    s.erase( std::remove( s.begin(), s.end(), ' ' ), s.end() );
    // Skip any comments
    if(s[0] == '#' || s.size() == 0){
      continue;
    }

    // Determine if we should optimize Nav Plan.
    if(s.find("Optimize") != std::string::npos){
      string a = s.substr(s.find("=")+1);
      std::stringstream ss(a);
      ss >> optimizeNavPlan;
    }

    //Save coordinates
    if(s == "BEGIN_COORDINATES"){
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
