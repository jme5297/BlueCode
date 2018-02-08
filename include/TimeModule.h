#pragma once
#include <chrono>
#include <vector>
#include <string>
#include <tuple>

namespace Times{

  using namespace std::chrono;
  using std::vector;
  using std::string;
  using std::tuple;

  class TimeModule{
  public:
    TimeModule();
    static void InitProccessCounter(string str, double dt);
    static void AddMilestone(string str);
    static bool ProccessUpdate(string str);
    static double GetElapsedTime(string str);
    static double GetProccessDelta(string str);
    static double GetLastProccessDelta(string str);
    #ifdef DEBUG
    static void SetTimeSimDelta(double d);
    static void Run();
    #endif

  protected:

    #ifndef DEBUG
    static std::vector< tuple<string, time_point<steady_clock> > > milestones;
    static std::vector< tuple<string, double, time_point<steady_clock>, double > > processes;
    #else
    static double currentSimTime;
    static double simDelta;
    static std::vector< tuple<string, double > > milestones;
    static std::vector< tuple<string, double, double, double > > processes;
    #endif
    static int FindMilestoneIndex(string str);
    static int FindProccessIndex(string str);

  };
}
