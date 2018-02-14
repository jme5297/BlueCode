#pragma once
#include <Parser.h>
#include <TimeModule.h>

#ifdef SIM
#include <PlantModel/PlantModel.h>
#endif

// If Camera use is defined, then include OpenCV header files here.
#ifdef USE_CAMERA
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#endif

namespace sensors {

#ifdef USE_CAMERA
	using namespace cv;
#endif

	/*!
	 * Main class for operating the connected camera. If USE_CAMERA compiling flag was enabled,
	 * then OpenCV will be called to take a photo of the first default camera connected to this
	 * device.
	 */
	class Camera {
	public:
		/// Main initializion routine for the Camera.
		bool Init();
		/// Main reset routine for the Camera class.
		bool Reset();
		/// In charge of enabling the connected camera and taking an image.
		bool TakeImage(int i);
	};

}
