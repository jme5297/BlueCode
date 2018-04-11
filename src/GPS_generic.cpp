#include <sensors/GPS/GPS_generic.h>
#include <string.h>

using namespace sensors;
using namespace Times;

#ifdef SIM
using namespace Plant;
#else
unsigned char rx_buffer[80];
void RunGPS();
void process();
void data_GR(char * buffer, int loc);
double lat, lon, cog, vel;
int uart0_filestream = -1;
#endif

Coordinate lastCoordinates;
Coordinate currentGPSCoordinates;
double vehicleHeading;
double vehicleVelocity;

GPS::GPS() {

}
GPS::~GPS() {

}
bool GPS::Init() {

#ifdef SIM
	PlantModel::GetVehicle()->InitializeGPS();
	currentGPSCoordinates = PlantModel::GetVehicle()->gps.coords;
	lastCoordinates = currentGPSCoordinates;
	TimeModule::InitProccessCounter("GPS", Parser::GetRefresh_GPS());
	vehicleHeading = 0.0;
	return PlantModel::GetVehicle()->gps.initialized;
#else

	uart0_filestream = open("/dev/ttyO4", O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
	if (uart0_filestream == -1)
	{
		//ERROR - CAN'T OPEN SERIAL PORT
		printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
		return false;
	}
	struct termios options;
	tcgetattr(uart0_filestream, &options);
	options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;		//<Set baud rate
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(uart0_filestream, TCIFLUSH);
	tcsetattr(uart0_filestream, TCSANOW, &options);
	if (uart0_filestream == -1)
	{
		return false;
	}

	// THIS IS TEMPORARY
	/*
	Coordinate c;
	c.lat = Parser::GetInitialLatitude();
	c.lon = Parser::GetInitialLongitude();
	currentGPSCoordinates = c;
	lastCoordinates = c;
	vehicleHeading = Parser::GetInitialHeading();
  */

	std::thread rungps (RunGPS);
	TimeModule::Log("GPS", "Initializing GPS thread...");
	rungps.detach();

#endif

	return true;
}

void GPS::Run() {
#ifdef SIM
	double PI = 3.14159265;
	if (TimeModule::ProccessUpdate("GPS")) {

		double dt = TimeModule::GetLastProccessDelta("GPS");

		// Calculate GPS.
		Coordinate c = PlantModel::GetVehicle()->gps.coords;
		c.lat += (-0.5 + ((double)rand() / (RAND_MAX))) * gpsUncertainty / latToM;
		c.lon += (-0.5 + ((double)rand() / (RAND_MAX))) * gpsUncertainty / lonToM;
		currentGPSCoordinates = c;

		// Calculate Ground Course (heading).
		Coordinate c1 = lastCoordinates;
		Coordinate c2 = PlantModel::GetVehicle()->gps.coords;
		double dx = c2.lon - c1.lon;
		double dy = c2.lat - c1.lat;
		double z = atan2(dy, dx) * 180.0 / PI;
		double head = 90.0 - z;

		// Vehicle velocity
		double vx = dx/dt*84397.32550370222;
		double vy = dy/dt*111049.88839989061;
		double vell = sqrt(vx*vx + vy*vy);
		vehicleVelocity = vell + (-0.5 + ((double)rand() / (RAND_MAX))) * Parser::GetGPSVelocityUncertainty();

		std::cout << std::to_string(c.lon) << ", " << std::to_string(c.lat) <<
			", " << std::to_string(vehicleHeading) << ", " << vehicleVelocity << "\n";

		// Heading
		head = (head < 0.0) ? 360.0 + head : head;
		vehicleHeading = head + (-0.5 + ((double)rand() / (RAND_MAX))) * Parser::GetGPSHeadingUncertainty();

		// Update
		lastCoordinates = PlantModel::GetVehicle()->gps.coords;
	}
#endif
}

#ifndef SIM
void RunGPS(){
	int i = 0;
	unsigned char temp[1];
	while(1){
		int len = read(uart0_filestream, &temp, 1);
		if(len > 0){
			if(temp[0] != '\n' && i < 80){
				rx_buffer[i] = temp[0];
				i++;
			}
			else{
				rx_buffer[i] = '\0';
				i=0;
				process();
				lastCoordinates = currentGPSCoordinates;
				std::cout << std::to_string(lat) << ", " << std::to_string(lon) << ", " << std::to_string(cog) << ", " << std::to_string(vel*0.514444) << "\n";
				if(fabs(lon) > 0.01 && fabs(lat) > 0.01){
					Coordinate c;
					c.lat = lat;
					c.lon = lon;
					currentGPSCoordinates = c;
					vehicleHeading = cog;
					vehicleVelocity = vel*0.514444;
				}
			}
		}
	}
}

void process(){
	char field[20];
	data_GR(field, 0);
	if (strcmp(field, "$GPRMC") == 0){

		data_GR(field, 3);
		double lat_nmea;
		double lat_degs;
		double lat_mins;
		lat_nmea = strtod(field,NULL);
		lat_nmea = lat_nmea / 100.0;
		lat_mins = lat_nmea - (double)((int)lat_nmea);
		lat_degs = lat_nmea - lat_mins;
		lat = lat_degs + lat_mins/0.60;

		data_GR(field, 5);
		double lon_nmea;
		double lon_degs;
		double lon_mins;
		lon_nmea = strtod(field,NULL);
		lon_nmea = lon_nmea / 100.0;
		lon_mins = lon_nmea - (double)((int)lon_nmea);
		lon_degs = lon_nmea - lon_mins;
		lon = -1.0*(lon_degs + lon_mins/0.60);

		data_GR(field, 7);
		vel = strtod(field, NULL);

		data_GR(field, 8);
		cog =  strtod(field,NULL);
	}
}

void data_GR(char * buffer, int loc){
	int sentencePos =0;
	int fieldPos = 0;
	int commacount =0;
	while (sentencePos < 80){
		if(rx_buffer[sentencePos] == ','){
			commacount ++;
			sentencePos ++;
		}
		if(commacount == loc){
			buffer[fieldPos] = rx_buffer[sentencePos];
			fieldPos++;
		}
		sentencePos++;
	}
	buffer[fieldPos] = '\0';
}
#endif

bool GPS::Reset() { return true; }
Coordinate GPS::GetCurrentGPSCoordinates() { return currentGPSCoordinates; }
double GPS::GetGPSGroundCourse() { return vehicleHeading; }
double GPS::GetGPSVelocity() { return vehicleVelocity; }
