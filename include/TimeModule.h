#include <preprocdef.h>
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
    void InitProccessCounter(string str, double dt);
    void AddMilestone(string str);
    bool ProccessUpdate(string str);
    double GetElapsedTime(string str);
    double GetProccessDelta(string str);
    double GetLastProccessDelta(string str);
    #ifdef DEBUG
    void SetTimeSimDelta(double d);
    void Run();
    #endif
    
  protected:

    #ifndef DEBUG
    std::vector< tuple<string, time_point<steady_clock> > > milestones;
    std::vector< tuple<string, double, time_point<steady_clock>, double > > processes;
    #else
    double currentSimTime;
    double simDelta;
    std::vector< tuple<string, double > > milestones;
    std::vector< tuple<string, double, double, double > > processes;
    #endif
    int FindMilestoneIndex(string str);
    int FindProccessIndex(string str);

  };
}
