#pragma once
#include <preprocdef.h>
#include <sensors/Sensors_generic.h>
#include <PlantModel/PlantModel.h>

#ifdef USE_CAMERA
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <libv4l2.h>
#endif

namespace sensors{
    
    
    // Main class for handling camera capabilities
    class Camera{
    public:
        Camera();
        ~Camera();
        bool Init();
        bool Reset();
        bool Enable();
        bool Disable();
        bool TakeImage(int argc, char **argv, int a);
        
        
    };
    
}
