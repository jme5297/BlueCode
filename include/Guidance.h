#pragma once
// #include <Navigation.h>
#include <Control.h>
#include <chrono>

namespace Guidance{

  // using namespace sensors;
  // using namespace Navigation;
  using namespace std::chrono;
  using namespace Control;

  enum ManeuverState{
		Calibrate,
		Maintain,
		AvoidDiverge,
		AvoidConverge,
		PayloadDrop,
		Complete
	};

	struct GuidanceManeuver{
	public:
		int index;
		ManeuverState state;
		system_clock::time_point beginGuidanceManeuver;
		system_clock::time_point endGuidanceManeuver;
		double turnToHeading;
		bool done;
	};

  class Guider{
  public:
    Guider();
    int GetGuidanceManeuverIndex();
		void RequestGuidanceManeuver(GuidanceManeuver cm);
		std::vector<GuidanceManeuver> GetGuidanceManeuverBuffer();
		GuidanceManeuver GetCurrentGuidanceManeuver();
		GuidanceManeuver currentGuidanceManeuver;
		void PayloadDrop(Controller& c);
    void Run(Controller& c);
  protected:
    int GuidanceManeuverIndex;
		std::vector<GuidanceManeuver> GuidanceManeuverBuffer;
  };
}
