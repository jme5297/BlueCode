#include <sensors/Camera/Camera_generic.h>
using namespace sensors;
using namespace Times;

Camera::Camera() {

}
Camera::~Camera() {

}
bool Camera::Init() {

#ifdef USE_CAMERA
	try{
		Webcam webcam("/dev/video0", Parser::GetCam_Width(), Parser::GetCam_Height());
	}catch(const char * msg){
		TimeModule::Log("CMA", "Can't access webcam...");
		return false;
	}
#endif

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

	Webcam webcam("/dev/video0", Parser::GetCam_Width(), Parser::GetCam_Height());
	auto frame = webcam.frame();
	std::ofstream image;
	image.open("image_" + std::to_string(a) + ".ppm");
	image << "P6\n" << Parser::GetCam_Width() << " " << Parser::GetCam_Height() << " 255\n";
	image.write((char *) frame.data, frame.size);
	image.close();

#endif

	return true;
}
