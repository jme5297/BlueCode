#pragma once
#include <Navigation.h>
#include <TimeModule.h>
#include <chrono>
#include <cmath>

/// All guidance-related classes and members
namespace Guidance{

  using namespace Navigation;
  using namespace std::chrono;
  using namespace Times;

  /*!
   * Encapsulates the type of guidance maneuver being performed.
   * There is a ManeuverState in every GuidanceManeuver struct that represents
   * what members of the manuever struct will be used.
   */
  enum ManeuverState{
		Calibrate,      ///< Vehicle is calibrating and should maintain the current heading and full speed.
                    /**< \note This may be unnecessary as Maintain could be used. */
		Maintain,       ///< Maintain current vehicle heading and speed.
    Turn,           ///< Vehicle is turning.
                    /**< This state could lead to several different control states:
                     * - **Track Vehicles**: Different motor speeds.
                     * - **Wheel Vehicles**: non-zero turning factor. */
		AvoidDiverge,   ///< Diverge away from an obstacle.
		AvoidConverge,  ///< Converge back to heading after avoiding an obstacle.
		PayloadDrop,    ///< Cut all speed and drop payload.
		Complete        ///< Maneuvering is complete.
                    /**< \note This state may not be necessary. */
	};

  /*! Represents a guidance maneuver. These guidance maneuvers are strung together
   * in a guidance maneuver buffer that represents all of the requested maneuvers
   * that the controller should operate on.
   */
	struct GuidanceManeuver{
	public:
		int index;            ///< Stores the guidance maneuver index from the guidance maneuver buffer.
		ManeuverState state;  ///< Maneuver state of this guidance maneuver.

		// steady_clock::time_point beginGuidanceManeuver;
		// steady_clock::time_point endGuidanceManeuver;

		int turnDirection;    ///< Direction of the vehicle turn.
                          /**<
                           * - **-1**: Left turn
                           * - **0**: Straight (used for non-turning maneuver states)
                           * - **1**: Right turn
                           * \note The Controller will handle turns differently based on the
                           * type of vehicle being operated. */
    double speed;         ///< Normalized requested control speed.
		bool done;            ///< Represents if the guidance maneuver has been completed.
                          /**< This value will be used by both the Guider and the Controller to
                           * determine when the next guidance maneuver in the guidance maneuver
                           * buffer should be analyzed. */
    double calibrationTime;   ///< Time to calibrate system in seconds.
    bool payloadDropComplete; ///< Flag that represents if a payload drop has been completed.

	};

  /// Main class for Guidance operations.
  class Guider{
  public:
    Guider();

    int GetGuidanceManeuverIndex(); ///< Returns the guidance maneuver index from the buffer being analyzed.
		void RequestGuidanceManeuver(GuidanceManeuver cm); ///< Request to add another guidance maneuver to the guidance maneuver buffer.
		std::vector<GuidanceManeuver> GetGuidanceManeuverBuffer(); ///<Return the entire guidance maneuver buffer.
		GuidanceManeuver& GetCurrentGuidanceManeuver(); ///< Get the current guidance maneuver from the maneuver buffer.

    void Run(Navigator& n); ///< Main execution function for the Guider.
    bool IsNavPlanComplete(); ///< Returns if all nav plan objectives have been acheived.

	void SetPayloadDistance(double);
	double GetPayloadDistance();

  protected:
    int GuidanceManeuverIndex; ///< Index of the current guidance maneuver being analyzed.

    /*!
     * Buffer of all guidance maneuvers. This buffer contains all information about the maneuvers that
     * have been performed, are being performed, and will be performed. New guidance maneuvers can be requested
     * by calling RequestGuidanceManeuver with a pre-constructed guidance maneuver.
     */
		std::vector<GuidanceManeuver> GuidanceManeuverBuffer;
    int coordinateIndex;  ///< The coordinate index of the NavPlan being targeted.
    bool isNavPlanComplete; ///< For determining if all of the nav plan objectives have been completed.

	double payloadDistance; ///< Required distance to reach when dropping payloads at locations.
  };
}
