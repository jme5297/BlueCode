#include <sensors/Camera/Camera_generic.h>

using namespace sensors;

#ifdef SIM
using namespace Plant;
#endif

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

/// @todo There is currently an issue with taking more than one image.
bool Camera::TakeImage(int i){

	#ifndef USE_CAMERA

	// No camera control for non-camera runs
	std::cout << "(Fake) Camera image taken!\n";
	return true;

	#else

	/// @todo This throws errors on some platforms.
	// capture.set(CV_CAP_PROP_FRAME_WIDTH,1920);
	// capture.set(CV_CAP_PROP_FRAME_HEIGHT,1080);
	if(!capture.isOpened()){
		capture = VideoCapture(0);
		return false;
	}
	Mat frame, edges;
	capture >> frame;
	if(frame.empty()){
		std::cout << "Failed to capture an image.\n";
		return false;
	}

	std::string img_name = "out/capture_" + std::to_string(i) + ".bmp";
	bool check = imwrite(img_name, frame);

	#ifdef SIM
	PlantModel::UpdateImage(img_name);
	#endif

	if(check)
	{
		std::cout << "Camera image taken!\n";
	}else
	{
		std::cout << "Failed to write image!\n";
	}

	#endif

	capture.release();
	return true;
}
