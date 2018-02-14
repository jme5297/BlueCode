#include <sensors/Camera/Camera_generic.h>

using namespace Times;
using namespace sensors;

#ifdef SIM
using namespace Plant;
#endif

#ifdef USE_CAMERA
using namespace cv;
#endif

bool Camera::Init(){

	return true;
}
bool Camera::Reset(){

	return true;
}

/// @todo There is currently an issue with taking more than one image.
bool Camera::TakeImage(int i){

	#ifndef USE_CAMERA

	// Print out a message that the camera (fake) has successfully taken an image.
	std::cout << "(Fake) Camera image taken!\n";
	return true;

	#else

	// Open up the default video capture device.
	VideoCapture capture = VideoCapture(0);
	capture.open(0);

	/// @todo This throws errors on some platforms. Ensure this works on game-day.
	capture.set(CV_CAP_PROP_FRAME_WIDTH,1920);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT,1080);

	// Save a single frame of the capture device.
	Mat frame, edges;
	capture >> frame;
	if(frame.empty()){
		std::cout << "Failed to capture an image.\n";
		return false;
	}

	// Save the image to the device and report on either its success or failure.
	std::string img_name = "out/capture_" + std::to_string(i) + ".bmp";
	bool check = imwrite(img_name, frame);
	if (check){
		std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "]: Camera image taken!\n";
	}else{
		std::cout << "[" << std::to_string(TimeModule::GetElapsedTime("BeginMainOpsTime")) << "]: Failed to write image!\n";
	}

	// If simulation mode is enabled, then update the displayed image on the screen->
	#ifdef SIM
	PlantModel::UpdateImage(img_name);
	#endif

	// Release the video capture.
	capture.release();
	VideoCapture* c = &capture;
	c = NULL;

	#endif // USE_CAMERA

	return true;
}
