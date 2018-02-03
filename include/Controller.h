#pragma once
#include <preprocdef.h>
#include <PlantModel/PlantModel.h>

class Controller{

public:
	Controller();
	~Controller();

	void SetMotorSpeeds(double speed);
	void SetMotorLSpeed(double speed);
	void SetMotorRSpeed(double speed);

	double GetMotorLSpeed();
	double GetMotorRSpeed();

protected:
	double motorLSpeed;
	double motorRSpeed;

};
