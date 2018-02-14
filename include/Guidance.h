#pragma once
#include <Navigation.h>

/// All guidance-related classes and members
namespace Guidance {

	using namespace Navigation;
	using namespace Times;

	/*!
	 * Encapsulates the type of guidance maneuver being performed.
	 * There is a ManeuverState in every GuidanceManeuver struct that represents
	 * what members of the manuever struct will be used.
	 */
	enum ManeuverState {
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
	};

	/*! Represents a guidance maneuver. These guidance maneuvers are strung together
	 * in a guidance maneuver buffer that represents all of the requested maneuvers
	 * that the controller should operate on.
	 */
	struct GuidanceManeuver {
	public:
		int index;            ///< Stores the guidance maneuver index from the guidance maneuver buffer.
		ManeuverState state;  ///< Maneuver state of this guidance maneuver.

		int turnDirection;    ///< Direction of the vehicle turn.
						  /**<
						   * - **-1**: Left turn
						   * - **0**: Straight (used for non-turning maneuver states)
						   * - **1**: Right turn
						   * \note The Controller will handle turns differently based on the
						   * type of vehicle being operated. */

		double requestedTurnAngle;	///< How much the vehicle should turn for a specific turn.
		double currentTurnAngle;	///< How much vehicle has already turned.

		bool hasBeganDiverging = false;
		double maintainTime;		///< Time (seconds) that vehicle should be maintained.

		double speed;				///< Normalized requested control speed.
		bool done;					///< Represents if the guidance maneuver has been completed.
									/**< This value will be used by both the Guider and the Controller to
										* determine when the next guidance maneuver in the guidance maneuver
										* buffer should be analyzed. */
		bool payloadDropComplete;	///< Flag that represents if a payload drop has been completed.
		bool payloadImageTaken;		///< Flag that saves if an image has been taken of the payload.
	};

	/// Main class for Guidance operations.
	class Guider {
	public:
		Guider();

		int GetGuidanceManeuverIndex(); ///< Returns the guidance maneuver index from the buffer being analyzed.
		void RequestGuidanceManeuver(GuidanceManeuver cm); ///< Request to add another guidance maneuver to the guidance maneuver buffer.
		std::vector<GuidanceManeuver> GetGuidanceManeuverBuffer(); ///<Return the entire guidance maneuver buffer.
		GuidanceManeuver& GetCurrentGuidanceManeuver(); ///< Get the current guidance maneuver from the maneuver buffer.

		void Run(Navigator* n); ///< Main execution function for the Guider.
		bool IsNavPlanComplete(); ///< Returns if all nav plan objectives have been acheived.

		void SetPayloadDropRadius(double d) { payloadDropRadius = d; }
		void SetOffAngleDeviate(double d) { offAngleDeviate = d; }
		void SetOffAngleAccepted(double d) { offAngleAccepted = d; }
		void SetCalibrationTime(double d) { calibrationTime = d; }
		void SetMinimumMaintainTime(double d) { minimumMaintainTime = d; }
		void SetObstacleDivergenceAngle(double d) { obstacleDivergenceAngle = d; }
		void SetObstacleDivergenceTime(double d) { obstacleDivergenceTime = d; }
		void SetPayloadServoTime(double d) { payloadServoTime = d; }
		void SetTurnFactorDPS(double d) { turnFactorDPS = d; }
		void SetMaxVehicleSpeed(double d) { maxVehicleSpeed = d; }

		double GetPayloadDropRadius() { return payloadDropRadius; }
		double GetPayloadServoTime() { return payloadServoTime; }

	protected:
		int GuidanceManeuverIndex; ///< Index of the current guidance maneuver being analyzed.

		double payloadDropRadius;
		double offAngleDeviate;
		double offAngleAccepted;
		double calibrationTime;
		double minimumMaintainTime;
		double obstacleDivergenceAngle;
		double obstacleDivergenceTime;
		double payloadServoTime;
		double turnFactorDPS;
		double maxVehicleSpeed;

		/*!
		 * Buffer of all guidance maneuvers. This buffer contains all information about the maneuvers that
		 * have been performed, are being performed, and will be performed. New guidance maneuvers can be requested
		 * by calling RequestGuidanceManeuver with a pre-constructed guidance maneuver.
		 */
		std::vector<GuidanceManeuver> GuidanceManeuverBuffer;
		int coordinateIndex;  ///< The coordinate index of the NavPlan being targeted.
		bool isNavPlanComplete; ///< For determining if all of the nav plan objectives have been completed.

		double latToM = 111050.0;
		double lonToM = 84397.0;
	};
}
