#include <sensors/Camera/Camera_generic.h>

using namespace sensors;

Camera::Camera(){

}
Camera::~Camera(){

}
bool Camera::Init(){

	return true;
}
bool Camera::Reset(){

	return true;
}
bool Camera::Enable(){

	return true;
}
bool Camera::Disable(){

	return true;
}
bool Camera::TakeImage(){

	#ifdef SIM
	// No camera control for SIM mode
	return true;
	#else
	// Actual code goes here to take Camera image
	return true;
	#endif

}
