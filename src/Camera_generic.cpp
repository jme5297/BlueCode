#include <sensors/Camera/Camera_generic.h>
using namespace sensors;
using namespace Times;
int plNum = 0;

Camera::Camera() {

}
Camera::~Camera() {

}
bool Camera::Init() {

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
	plNum++;

	std::string cmd = "./takePhoto " + std::to_string(plNum);
	system(cmd.c_str());

	return true;
#endif
}


