#include <iostream>
#include <vector>

namespace sensors{
	namespace GPS {

		struct Coordinate {
			double lat;
			double lon;
		};

		Coordinate GetCurrentGPSCoordinates();
		
	}
}
