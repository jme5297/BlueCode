#include <sensors/Laser/Laser_generic.h>

using namespace sensors;

#ifdef SIM
using namespace Plant;
#endif

Laser::Laser(int i){
	ID = i;
}
Laser::~Laser(){

}
bool Laser::Init(){

	return true;
}
bool Laser::Reset(){

	return true;
}
bool Laser::ReadLaser(){

	#ifdef SIM // sim mode
	Vehicle* v = PlantModel::GetVehicle();
	return v->lasers[ID].val;
	#else
	// ACTUAL laser information here
	int digit = -1;
    ifstream laser;
    laser.open("/sys/class/gpio/gpio60/value");

    if (laser.is_open())
    {
        while (laser.good() )
        {
            laser >> digit;
        }
    }
    
    // laser.close();
    else
    {
        cout << "Unable to read the laser data\n";
        return false;
    }

    if (digit == 0) //obstacle detected
    {
        cout<<"Laser sensor: obstacle detected!"<<endl;
				return true;
    }
    else if (digit == 1) //no obstacle detect
    {
        // cout<<" No obstacle detected"<<endl;
				return false;
    }

	return 0.0;
	#endif
}
