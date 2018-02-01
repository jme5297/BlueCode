#include <sensors/GPS/GPS_generic.h>

namespace sensors{
	namespace GPS{

		Coordinate GetCurrentGPSCoordinates(){

			Coordinate cd;
			cd.lat = 0.0;
			cd.lon = 0.0;

			return cd;
		}
		
	}	
}

