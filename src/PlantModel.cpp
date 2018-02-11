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
IGUIStaticText* vehicleInfo;
ISceneNode* vehicleNode;
IMeshSceneNode* vehicleModel;

void PlantModel::Initialize(){
  std::cout << "PLANT INITIALIZED " << GetVehicle()->gps.coords.lon << "\n\n";
  initTime = std::chrono::system_clock::now();
  fp_run = true;

  // Determine screen resolution.
  IrrlichtDevice* nulldev = createDevice(video::EDT_NULL);
  dimension2d<u32> deskres = nulldev->getVideoModeList()->getDesktopResolution();
  nulldev->drop();

#ifdef __APPLE__
  device = createDevice( video::EDT_OPENGL, dimension2d<u32>(deskres.Width / 2, deskres.Height / 2), 16, false, false, false, 0);
#else
  device = createDevice( video::EDT_OPENGL, dimension2d<u32>(deskres.Width / 2, deskres.Height / 2), 16, false, false, false, 0);
#endif

   path p = device->getFileSystem()->getWorkingDirectory();

  if (!device)
    return;
  device->setWindowCaption(L"Giving You Something Nice To Look At");
  driver = device->getVideoDriver();
  smgr = device->getSceneManager();
  guienv = device->getGUIEnvironment();

  IAnimatedMesh* mesh = smgr->getMesh("irrlicht/media/car.obj");
  // If mesh doesn't exist, go a few directories backwards.
  for(int i = 0; i < 3; i++)
  {
	  if (!mesh)
	  {
		  device->getFileSystem()->changeWorkingDirectoryTo("..");
		  mesh = smgr->getMesh("irrlicht/media/car.obj");
	  }
  }

  // If mesh still doesn't exist, we have a problem.
  if (!mesh) {
	  device->drop();
	  return;
  }

  vehicleModel = smgr->addMeshSceneNode( mesh );
  if (vehicleModel)
  {
	  vehicleModel->setMaterialFlag(EMF_LIGHTING, false);
	  // vehicleModel->setMD2Animation(scene::EMAT_STAND);
	  // vehicleModel->setMaterialTexture( 0, driver->getTexture("irrlicht/media/sydney.bmp") );
	  vehicleModel->setScale(vector3df(0.5, 0.5, 0.5));
  }
  vehicleNode = smgr->addEmptySceneNode();
  vehicleNode->addChild(vehicleModel);

  smgr->addCameraSceneNode(0, vector3df(0,20,-10), vector3df(0,0,0));

  dimension2du screenSize = driver->getScreenSize();
  s32 offset = 10;

  IGUISkin* skin = guienv->getSkin();
  IGUIFont* font = guienv->getFont("irrlicht/media/fonthaettenschweiler.bmp");
  skin->setFont(font);
  skin->setFont(guienv->getBuiltInFont(), EGDF_TOOLTIP);

  // Add vehicle information.
  vehicleInfo = guienv->addStaticText(L"Vehicle Information:",
	  rect<s32>((s32)(screenSize.Width * 0.75), (s32)(screenSize.Height * 0.75), screenSize.Width-10, screenSize.Height-10), true);
}

void PlantModel::Cleanup(){
  device->drop();
}

void PlantModel::UpdateImage(std::string str)
{
	ITexture* recentImage;

	recentImage = driver->getTexture(str.c_str());

	dimension2du screenSize = driver->getScreenSize();
	s32 offset = 10;

  IGUIImage * myImage;
  if(recentImage){
    myImage = guienv->addImage(rect<s32>(offset, offset, offset + screenSize.Width / 4 , offset + screenSize.Height / 4));
    myImage->setImage(recentImage);
    myImage->setScaleImage(true);
    myImage->setVisible(true);
  }
}

// Really, really simple physics model.
void PlantModel::Run(double dt){

  double dx;
  double dy;

	if(device->run())
	{
		double lat = GetVehicle()->gps.coords.lat;
		double lon = GetVehicle()->gps.coords.lon;
		double head = GetVehicle()->heading;
		double spd = GetVehicle()->wheelSpeedN;
		double str = GetVehicle()->wheelSteeringN;
		std::wstring txt = L"VEHICLE INFO";
		txt.append(L"\nLat: " + std::to_wstring(lat));
		txt.append(L"\nLon: " + std::to_wstring(lon));
		txt.append(L"\nHead: " + std::to_wstring(head));
		txt.append(L"\nWheel Speed: " + std::to_wstring(spd));
		txt.append(L"\nWheel Steering: " + std::to_wstring(str));
		vehicleInfo->setText(txt.c_str());

		vehicleNode->setPosition(vector3df(lon, 0, lat));
		vehicleNode->setRotation(vector3df(0, head, 0));

		SMaterial m;
		m.Lighting = false;
		driver->setMaterial(m);
		driver->setTransform(video::ETS_WORLD, core::IdentityMatrix);

		driver->beginScene(true, true, SColor(255,100,101,140));
		smgr->drawAll();
		guienv->drawAll();

		driver->draw3DLine(
			core::vector3df(-10.0f, 0.0f, 0.0f),
			core::vector3df(10.0f, 0.0f, 0.0f),
			SColor(255, 255, 255, 255)
		);
		driver->draw3DLine(
			core::vector3df(0.0f, 0.0f, -10.0f),
			core::vector3df(0.0f, 0.0f, 10.0f),
			video::SColor(255, 255, 255, 255)
		);

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
