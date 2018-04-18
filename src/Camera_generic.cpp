#include <sensors/Camera/Camera_generic.h>
using namespace sensors;
using namespace Times;
int plNum = 0;
std::ifstream errorReader;
int errorOffset = 0;

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
	errorReader.open("camErrorOffset.txt");
	errorReader >> errorOffset;
	std::cout << "Current error count: " << errorOffset << "\n";
	errorReader.close();

	TimeModule::Log("CMA","Sleeping to relax...");
	usleep(1000000);

	std::string cmd = "fswebcam " //-r " + std::to_string(Parser::GetCam_Width()) + "x" + std::to_string(Parser::GetCam_Height()) + 
		" -d /dev/video" + std::to_string(Parser::GetCam_Device() + errorOffset) +  " image_" + std::to_string(plNum) + ".jpg";

	system(cmd.c_str());

	return true;
#endif
}


