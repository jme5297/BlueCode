#include <sensors/Camera/Camera_generic.h>

using namespace sensors;
using namespace Plant;

#ifdef USE_CAMERA
using namespace cv;
#endif

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
bool Camera::TakeImage(int i){

	#ifndef USE_CAMERA

	// No camera control for non-camera runs
	std::cout << "(Fake) Camera image taken!\n";
	return true;

	#else

	// Actual code goes here to take Camera image
	VideoCapture capture(0);
	capture.set(CV_CAP_PROP_FRAME_WIDTH,1920);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT,1080);
	if(!capture.isOpened()){
	std:: cout << "Failed to connect to the camera.\n";
		return false;
	}
	Mat frame, edges;
	capture >> frame;
	if(frame.empty()){
		std::cout << "Failed to capture an image.\n";
		return false;
	}

	std::string img_name = "out/capture_" + std::to_string(i) + ".png";
	imwrite(img_name, frame);
	PlantModel::UpdateImage(img_name);
	std::cout << "Camera image taken!\n";

	#endif

	return true;
}
