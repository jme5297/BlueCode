#include <iostream>
#include <vector>
#include <tuple>

namespace sensors{
	namespace GPS {

		struct Coordinate {
			double lat;
			double lon;
		};

		Coordinate GetCurrentGPSCoordinates();
		
	}
}
