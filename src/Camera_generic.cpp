#include <sensors/Camera/Camera_generic.h>
using namespace sensors;
using namespace Times;
int imagecount;

Camera::Camera() {

}
Camera::~Camera() {

}
bool Camera::Init() {

	imagecount = 1;
	return true;
}
bool Camera::Reset() {

	return true;
}
bool Camera::Enable() {

	return true;
}
bool Camera::Disable() {

	return true;
}
bool Camera::TakeImage(int a) {

#ifndef USE_CAMERA
	std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "][CMA]: (Fake) Camera image taken!\n";
	return true;
#else
	std::string command =
		"fswebcam --font :16 --bottom-banner --title 'Payload Drop #" + std::to_string(imagecount) +
		"' --subtitle 'Guidance Maneuver #" + std::to_string(a) +
		"' -r " + std::to_string(Parser::GetCam_Width()) + "x" + std::to_string(Parser::GetCam_Height()) +
		" image_" + std::to_string(imagecount) + ".jpg";
		std::cout << command << "\n";
	system(command.c_str());
#endif
	imagecount++;
	return true;
}
