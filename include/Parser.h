#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <sensors/Sensors_generic.h>

using namespace std;
using namespace sensors;

class Parser{

public:
  static void ReadInputs(string file);
  static std::vector<Coordinate> GetInputCoordinates(){ return inputCoords; }
  static bool OptimizeNavPlan(){ return optimizeNavPlan; }

protected:
  static std::ifstream configFile;
  static std::vector<Coordinate> inputCoords;
  static bool optimizeNavPlan;

};
