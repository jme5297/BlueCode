#define PI 3.14159265
#include <PlantModel/PlantModel.h>
#include <iostream>
#include <string>

using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

using namespace sensors;
using namespace Plant;

// Declare statics
Vehicle PlantModel::veh;
std::chrono::time_point<std::chrono::system_clock> PlantModel::initTime;
std::chrono::time_point<std::chrono::system_clock> PlantModel::lastRunCall;
bool PlantModel::fp_run;
IrrlichtDevice* PlantModel::device;
IVideoDriver* PlantModel::driver;
ISceneManager* PlantModel::smgr;
IGUIEnvironment* PlantModel::guienv;
ITexture* PlantModel::recentImage;

void PlantModel::Initialize(){
  std::cout << "PLANT INITIALIZED " << GetVehicle()->gps.coords.lon << "\n\n";
  initTime = std::chrono::system_clock::now();
  fp_run = true;

#ifdef __APPLE__
  device = createDevice( video::EDT_OPENGL, dimension2d<u32>(640, 480), 16, false, false, false, 0);
#else
  device = createDevice( video::EDT_SOFTWARE, dimension2d<u32>(640, 480), 16, false, false, false, 0);
#endif

   path p = device->getFileSystem()->getWorkingDirectory();

#ifndef __APPLE__
  device->getFileSystem()->changeWorkingDirectoryTo("..");
#endif

  if (!device)
    return;
  device->setWindowCaption(L"Giving You Something Nice To Look At");
  driver = device->getVideoDriver();
  smgr = device->getSceneManager();
  guienv = device->getGUIEnvironment();
  guienv->addStaticText(L"Hello World! This is the Irrlicht Software renderer!",
    rect<s32>(10,10,260,22), true);


  IAnimatedMesh* mesh = smgr->getMesh("irrlicht/media/sydney.md2");
  if (!mesh)
  {
    device->drop();
    return;
  }
  IAnimatedMeshSceneNode* node = smgr->addAnimatedMeshSceneNode( mesh );
  if (node)
  {
    node->setMaterialFlag(EMF_LIGHTING, false);
    node->setMD2Animation(scene::EMAT_STAND);
    node->setMaterialTexture( 0, driver->getTexture("irrlicht/media/sydney.bmp") );
  }

  smgr->addCameraSceneNode(0, vector3df(0,30,-40), vector3df(0,5,0));
  std::cout << "added cam.\n";

  driver->getMaterial2D().AntiAliasing = video::EAAM_FULL_BASIC;
}

void PlantModel::Cleanup(){
  device->drop();
}

void PlantModel::UpdateImage(std::string str)
{
	ITexture* recentImage;
#ifdef _WIN32
	recentImage = driver->getTexture(str.c_str());
#else
	recentImage = driver->getTexture(("build/" + str).c_str());
#endif
}

// Really, really simple physics model.
void PlantModel::Run(double dt){

  double dx;
  double dy;

	if(device->run())
	{
		driver->beginScene(true, true, SColor(255,100,101,140));
		smgr->drawAll();
		guienv->drawAll();
		driver->endScene();
	}

  switch(GetVehicle()->vehicleType){
    //-------------------------
    //    TRACK MODE
    //-------------------------
    case VehicleType::Track:{
      double speedL = GetVehicle()->motL.val * GetVehicle()->maxSpeedMPS;
      double speedR = GetVehicle()->motR.val * GetVehicle()->maxSpeedMPS;
      // Angle is in RADIANS
      double dtheta = (speedL - speedR) / GetVehicle()->width * dt;
      // Update heading if the vehicle is turning.
      GetVehicle()->heading += dtheta * 180.0 / PI ;
      GetVehicle()->heading = fmod(GetVehicle()->heading, 360.0);

      // Turning radius of the INSIDE track.
      double r;
      if(dtheta <= 1e-15){
        r = -1.0; // turning radius is infinity here.
        dx = speedL*dt*sin(GetVehicle()->heading * PI / 180.0);
        dy = speedL*dt*cos(GetVehicle()->heading * PI / 180.0);
      }else{
        r = speedR * dt / dtheta;
        dx = (r + GetVehicle()->width/2.0)*dtheta*sin(GetVehicle()->heading * PI / 180.0);
        dy = (r + GetVehicle()->width/2.0)*dtheta*cos(GetVehicle()->heading * PI / 180.0);
      }
      break;
    }
    //-------------------------
    //    WHEEL MODE
    //-------------------------
    case VehicleType::Wheel:
      // http://www.davdata.nl/math/turning_radius.html
      // This is the dav-data model for a bicycle, but it should work well enough for us.
      // NOTE: This is ideal for the turning raidus of a bicycle. There may have to be a more
      // complex model used if this is not sufficient.
      double w  = GetVehicle()->length;
      double alpha = GetVehicle()->wheelSteeringN * GetVehicle()->maxWheelSteeringAngleDeg;
      double speed = GetVehicle()->wheelSpeedN * GetVehicle()->maxSpeedMPS;

      if(fabs(alpha) <= 1e-15){
        dx = speed*dt*sin(GetVehicle()->heading * PI / 180.0);
        dy = speed*dt*cos(GetVehicle()->heading * PI / 180.0);
      }else{
        double R = w / sin(fabs(alpha) * PI / 180.0);
        double r = w / tan(fabs(alpha) * PI / 180.0);
        // Calculate the approximate centroid radius.
        double wid = GetVehicle()->width;
        double R_cent = 0.5*(0.5*(R+R+wid) + 0.5*(r+r+wid));
        double dtheta = speed * dt / R_cent; // radians

        if(alpha < 0.0){
          GetVehicle()->heading -= dtheta * 180.0 / PI ;
        }else{
          GetVehicle()->heading += dtheta * 180.0 / PI ;
        }
        GetVehicle()->heading = fmod(GetVehicle()->heading, 360.0);

        dx = (R_cent)*dtheta*sin(GetVehicle()->heading * PI / 180.0);
        dy = (R_cent)*dtheta*cos(GetVehicle()->heading * PI / 180.0);
      }
      break;
  }

  // Update the vehicle position.
  GetVehicle()->gps.coords.lon += dx;
  GetVehicle()->gps.coords.lat += dy;

  return;
}
void PlantModel::PrintStatus(){
  std::cout << "t: " << GetElapsedSeconds() <<
    " --- lat: " << std::to_string(GetVehicle()->gps.coords.lat) << ", lon: " << GetVehicle()->gps.coords.lon << "\n";
}
std::chrono::duration<double> PlantModel::GetSimDuration(){
  return std::chrono::system_clock::now()-initTime;
}
double PlantModel::GetElapsedSeconds(){
  std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now()-initTime;
  return elapsed_seconds.count();
}

Vehicle * PlantModel::GetVehicle(){
  return &veh;
}
