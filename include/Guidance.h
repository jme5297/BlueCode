#pragma once
#include <Navigation.h>
#include <chrono>
#include <cmath>

/// All guidance-related classes and members
namespace Guidance{

  using namespace Navigation;
  using namespace std::chrono;

  enum ManeuverState{
		Calibrate,
		Maintain,
    Turn,
		AvoidDiverge,
		AvoidConverge,
		PayloadDrop,
		Complete
	};

	struct GuidanceManeuver{
	public:
		int index;
		ManeuverState state;
		steady_clock::time_point beginGuidanceManeuver;
		steady_clock::time_point endGuidanceManeuver;
		int turnDirection;
    double speed;
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
		void PayloadDrop();
    void Run(Navigator& n);
    bool IsNavPlanComplete();
  protected:
    int GuidanceManeuverIndex;
		std::vector<GuidanceManeuver> GuidanceManeuverBuffer;
    int coordinateIndex;
    bool isNavPlanComplete;
  };
}
