#pragma once
#include <preprocdef.h>
#include <vector>
#include <PlantModel/PlantModel.h>
#include <chrono>

namespace Control{

	using namespace std::chrono;

	enum VehicleMode{
		Track,
		Wheel
	};

	enum ControlState{
		Calibrate,
		Maintain,
		AvoidDiverge,
		AvoidConverge,
		PayloadDrop,
		Complete
	};

	struct ControlMove{
	public:
		int index;
		ControlState state;
		system_clock::time_point beginControlMove;
		system_clock::time_point endControlMove;
		double turnToHeading;
		bool done;
	};

	class Controller{
	public:
		Controller();
		~Controller();

		// These assume track mode
		void SetMotorSpeeds(double speed);
		void SetMotorLSpeed(double speed);
		void SetMotorRSpeed(double speed);
		double GetMotorLSpeed();
		double GetMotorRSpeed();

		// These assume wheel mode
		void SetWheelSpeed(double speed);
		void SetWheelSteering(double steering);
		double GetWheelSpeed();
		double GetWheelSteering();

		// Main execution command
		void Run();

		int GetControlMoveIndex();
		void RequestControlMove(ControlMove cm);
		std::vector<ControlMove> GetControlMoveBuffer();
		ControlMove GetCurrentControlMove();

		VehicleMode currentVehicleMode;
		ControlMove currentControlMove;

		// Controls
		void PayloadDrop();

	protected:
		// For track system
		double motorLSpeed;
		double motorRSpeed;

		// For wheel system
		double wheelSpeed;
		double wheelSteering;

		int controlMoveIndex;
		std::vector<ControlMove> controlMoveBuffer;

	};
}
