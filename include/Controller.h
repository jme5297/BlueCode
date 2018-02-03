#pragma once
#include <preprocdef.h>
#include <vector>
#include <PlantModel/PlantModel.h>

namespace Control{

	enum ControlMode{
		Track,
		Wheel
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
		void RequestControlMove();

		ControlMode currentControlMode;

	protected:
		// For track system
		double motorLSpeed;
		double motorRSpeed;

		// For wheel system
		double wheelSpeed;
		double wheelSteering;

		int controlMoveIndex;

	};
}
