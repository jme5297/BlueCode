#define PI 3.14159265
#include <PlantModel/PlantModel.h>

using namespace Times;
using namespace sensors;
using namespace Plant;
Vehicle PlantModel::veh;
std::chrono::time_point<std::chrono::system_clock> PlantModel::initTime;

double PlantModel::initLon;
double PlantModel::initLat;
double PlantModel::latToM = 111050.0;
double PlantModel::lonToM = 84397.0;

// ---------------------
//		IRRLICHT
// ---------------------
#ifdef USE_IRRLICHT
using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;
IrrlichtDevice* device;
IVideoDriver* driver;
ISceneManager* smgr;
IGUIEnvironment* guienv;
ITexture* recentImage;
IGUIStaticText* vehicleInfo;
ISceneNode* vehicleNode;
IAnimatedMeshSceneNode* vehicleModel;
ICameraSceneNode* mainCam;
ILightSceneNode* mainLight;
std::vector<IMeshSceneNode*> payloads;
std::vector<IMeshSceneNode*> obstacles;
vector3df m_Rot;                   // H/V Position of camera on sphere (only X/Y used)
f32 m_Rad;                         // Radius of sphere
bool m_Dragging;                   // Is currently dragging?
vector2df m_DragStart;             // 2D Position on screen where the drag started
vector3df m_DragStartRotation;     // Rotation when drag started
IAnimatedMeshSceneNode* terrain;
ITriangleSelector* selector;
IMeshSceneNode * error;
Coordinate GPScoords;
double GPSheading;
bool deviceDropped;
#endif

/*! Initializes the PlantModel class.
* This function is also in charge of initializing all Irrlicht capabilities.
*/
void PlantModel::Initialize() {

	veh.Initialize();

	// Store the system time of the plant model initialization->
	initTime = std::chrono::system_clock::now();

	// Store the request initial latitude and longitutde
	initLat = Parser::GetInitialLatitude();
	initLon = Parser::GetInitialLongitude();

#ifdef USE_IRRLICHT

	// Determine screen resolution and create the main Irrlicht device.
	device = createDevice(video::EDT_OPENGL, dimension2d<u32>(1280, 720), 16, false, false, false, this);
	if (!device) { return; }
	device->setWindowCaption(L"BlueCode Simulator");
	device->setResizable(true);
	deviceDropped = false;

	// This is meant for debugging to see what directory Irrlicht starts in->
	path p = device->getFileSystem()->getWorkingDirectory();

	// Store references to the video driver, scene manager, and the GUI.
	driver = device->getVideoDriver();
	smgr = device->getSceneManager();
	guienv = device->getGUIEnvironment();

	/// \note While it is necessary to change working directory, I will NOT be using the model for the car (for now)
	IAnimatedMesh* mesh = smgr->getMesh("irrlicht/media/car.obj");
	// Different machines start at at a different directory level. This is a fix for this issue.
	for (int i = 0; i < 3; i++) {
		if (!mesh) {
			device->getFileSystem()->changeWorkingDirectoryTo("..");
			mesh = smgr->getMesh("irrlicht/media/car.obj");
		}
	}

	// If mesh still doesn't exist, we have a problem.
	if (!mesh) {
		device->drop();
		return;
	}

	// Create a scene node for the vehicle.
	vehicleModel = smgr->addAnimatedMeshSceneNode(mesh);
	vehicleModel->setPosition(vector3df(0, veh.height*0.5, 0));
	vehicleModel->setRotation(vector3df(0, -(90.0 - veh.heading), 0));
	vehicleModel->setID(99);
	selector = smgr->createTriangleSelector(vehicleModel->getMesh(), vehicleModel);
	vehicleModel->setTriangleSelector(selector);
	selector->drop();

	// Add a camera to the scene.
	mainCam = smgr->addCameraSceneNode(0, vector3df(0, 30, 0), vehicleModel->getPosition());
	mainCam->setTarget(vehicleModel->getPosition());
	mainCam->bindTargetAndRotation(true);
	m_Rad = 7;
	m_Dragging = false;
	m_Rot.Y = PI;
	m_Rot.X = PI / 4.0;

	// Get the screen size.
	dimension2du screenSize = driver->getScreenSize();
	s32 offset = 10;

	// Update the font to be used.
	IGUISkin* skin = guienv->getSkin();
	IGUIFont* font = guienv->getFont("irrlicht/media/Gui1.xml");
	skin->setFont(font);
	skin->setFont(guienv->getBuiltInFont(), EGDF_TOOLTIP);

	// Add vehicle information
	vehicleInfo = guienv->addStaticText(L"Vehicle Information:",
		rect<s32>((s32)(screenSize.Width * 0.75), (s32)(screenSize.Height * 0.75), screenSize.Width - 10, screenSize.Height - 10), true);
	vehicleInfo->setBackgroundColor(SColor(100, 255, 255, 255));

	// Add terrain
	terrain = smgr->addAnimatedMeshSceneNode(smgr->getMesh("irrlicht/media/field.obj"));
	terrain->setRotation(vector3df(0, -226.0, 0));
	terrain->setMaterialTexture(0, driver->getTexture("irrlicht/media/field.png"));
	terrain->setMaterialFlag(EMF_LIGHTING, false);
	selector = smgr->createTriangleSelector(terrain->getMesh(), terrain);
	terrain->setTriangleSelector(selector);
	selector->drop();
	terrain->setID(0);

	// Add lights.
	mainLight = smgr->addLightSceneNode(0, vector3df(10, 10, 10), SColorf(0.1, 0.1, 0.1));
	mainLight->setLightType(E_LIGHT_TYPE::ELT_DIRECTIONAL);
	smgr->setAmbientLight(SColorf(0.2, 0.2, 0.2));

	// Add skybox.
	scene::ISceneNode* skydome = smgr->addSkyDomeSceneNode(driver->getTexture("irrlicht/media/skydome.jpg"), 16, 8, 0.95f, 2.0f);

	// Draw GPS circle error
	error = smgr->addSphereSceneNode(Parser::GetGPSUncertainty()*0.5, 32);
	error->getMaterial(0).Wireframe = true;
	error->setMaterialFlag(EMF_LIGHTING, false);
	smgr->getMeshManipulator()->setVertexColors(error->getMesh(), SColor(255, 0, 100, 0));
	error->getMaterial(0).NormalizeNormals = true;
	error->setScale(vector3df(1.0, 0.001, 1.0));

#endif
}

#ifdef USE_IRRLICHT
void PlantModel::DrawPayloadLocations(std::vector<Coordinate> coords, double pldist)
{
	// Draw all of the payload locations.
	for (int i = 0; i < coords.size(); i++) {
		payloads.push_back(smgr->addSphereSceneNode(pldist));
		payloads[i]->getMaterial(0).Wireframe = true;
		payloads[i]->setMaterialFlag(EMF_LIGHTING, false);
		smgr->getMeshManipulator()->setVertexColors(payloads[i]->getMesh(), SColor(255, 255, 255, 0));
		payloads[i]->getMaterial(0).NormalizeNormals = true;
		payloads[i]->setPosition(vector3df(LonToX(coords[i].lon), 0, LatToZ(coords[i].lat)));
	}
}
void PlantModel::DrawObstacles(std::vector<Obstacle> obs)
{
	// Draw all of the obstacles.
	for (int i = 0; i < obs.size(); i++) {
		obstacles.push_back(
			smgr->addCubeSceneNode(
				1.0,
				0,
				-1,
				vector3df(LonToX(obs[i].lon), 0.5*obs[i].height, LatToZ(obs[i].lat)),	// Position
				vector3df(0, obs[i].yRotation, 0),	// Rotation
				vector3df(obs[i].length, obs[i].height, obs[i].width)));	// Scale
		smgr->getMeshManipulator()->setVertexColors(obstacles[i]->getMesh(), SColor(255, 0, 100, 255));
		obstacles[i]->setMaterialFlag(EMF_LIGHTING, false);
		obstacles[i]->getMaterial(0).Wireframe = true;
		obstacles[i]->getMaterial(0).NormalizeNormals = true;
		obstacles[i]->setID(0);
		selector = smgr->createTriangleSelector(obstacles[i]->getMesh(), obstacles[i]);
		obstacles[i]->setTriangleSelector(selector);
		selector->drop();
	}
}
void PlantModel::SendGPSData(Coordinate c, double h) {
	GPScoords = c;
	GPSheading = h;
}
/**
 * Updates the image being displayed on the corner of the Irrlicht window. This functionality
 * was added to ensure that pictures were being taken by the camera if USE_CAMERA was defined.
 * @param[in] str - String representing file & path location of the image to load.
 */
void PlantModel::UpdateImage(std::string str)
{
	if (!deviceDropped) {
		// Pull in the image as a texture.
		ITexture* recentImage;
		recentImage = driver->getTexture(str.c_str());

		// If texture creation successful, add to the screen->
		if (recentImage) {
			dimension2du screenSize = driver->getScreenSize();
			s32 offset = 10;
			IGUIImage * myImage = guienv->addImage(rect<s32>(offset, offset, offset + screenSize.Width / 4, offset + screenSize.Height / 4));
			myImage->setImage(recentImage);
			myImage->setScaleImage(true);
			myImage->setVisible(true);
		}
	}

}
#endif

void PlantModel::Cleanup() {

#ifdef USE_IRRLICHT
	if (!deviceDropped) {
		device->drop();
		deviceDropped = true;
	}
#endif

}

/**
 * This class handles the physics update of the plant model.
 */
void PlantModel::Run(double dt) {

	double dx;
	double dy;

	// Perform physics updates manually for updating the position of the vehicle.
	// http://www.davdata.nl/math/turning_radius.html
	// This is the dav-data model for a bicycle, but it should work well enough for us.
	// NOTE: This is ideal for the turning raidus of a bicycle. There may have to be a more
	// complex model used if this is not sufficient.
	double w = GetVehicle()->length;
	double alpha = GetVehicle()->wheelSteeringN * GetVehicle()->maxWheelSteeringAngleDeg;
	double speed = GetVehicle()->wheelSpeedN * GetVehicle()->maxSpeedMPS;

	// Add a terrain roughness factor
	speed = speed *  Parser::GetTerrainRoughness() + (0.1*speed*sin(GetVehicle()->heading * PI / 180.0));

	if (fabs(alpha) <= 1e-15) {
		dx = speed * dt*sin(GetVehicle()->heading * PI / 180.0);
		dy = speed * dt*cos(GetVehicle()->heading * PI / 180.0);
	}
	else {
		double R = w / sin(fabs(alpha) * PI / 180.0);
		double r = w / tan(fabs(alpha) * PI / 180.0);
		// Calculate the approximate centroid radius.
		double wid = GetVehicle()->width;
		double R_cent = 0.5*(0.5*(R + R + wid) + 0.5*(r + r + wid));
		double dtheta = speed * dt / R_cent; // radians

		if (alpha < 0.0) {
			GetVehicle()->heading -= dtheta * 180.0 / PI;
		}
		else {
			GetVehicle()->heading += dtheta * 180.0 / PI;
		}
		GetVehicle()->heading = fmod(GetVehicle()->heading, 360.0);

		dx = (R_cent)*dtheta*sin(GetVehicle()->heading * PI / 180.0);
		dy = (R_cent)*dtheta*cos(GetVehicle()->heading * PI / 180.0);
	}

	// Update the vehicle position->
	GetVehicle()->gps.coords.lon += dx / lonToM;
	GetVehicle()->gps.coords.lat += dy / latToM;

	// Update the Irrlicht engine.
#ifdef USE_IRRLICHT
	if (!deviceDropped) {
		UpdateEngine();
	}
#endif

	return;
}

/**
 * Handles updating of the Irrlicht engine. Some information is also passed back
 * to the vehicle (ex: obstacle detection).
 */
#ifdef USE_IRRLICHT
void PlantModel::UpdateEngine()
{
	/// @todo Determine if this conditional is necessary.
	if (device->run())
	{
		// Display the vehicle information on the screen->
		double lat = GetVehicle()->gps.coords.lat;
		double lon = GetVehicle()->gps.coords.lon;
		double head = GetVehicle()->heading;
		double spd = GetVehicle()->wheelSpeedN;
		double str = GetVehicle()->wheelSteeringN;
		std::wstring txt = L"";
		txt.append(L"Sim Time: " + std::to_wstring(TimeModule::GetElapsedTime("BeginMainOpsTime")));
		txt.append(L"\n--------------");
		txt.append(L"\nLat: " + std::to_wstring(lat));
		txt.append(L"\nLon: " + std::to_wstring(lon));
		txt.append(L"\nHead: " + std::to_wstring(head));
		txt.append(L"\nWheel Speed: " + std::to_wstring(spd));
		txt.append(L"\nWheel Steering: " + std::to_wstring(str));
		vehicleInfo->setText(txt.c_str());

		dimension2du screenSize = driver->getScreenSize();
		vehicleInfo->setRelativePosition(position2di((s32)(screenSize.Width*0.75), (s32)(screenSize.Height*0.75)));
		vehicleInfo->setMaxSize(dimension2du((u32)(screenSize.Width*0.25 - 10), (u32)(screenSize.Height*0.25 - 10)));
		vehicleInfo->setMinSize(dimension2du((u32)(screenSize.Width*0.25 - 10), (u32)(screenSize.Height*0.25 - 10)));

		// Update vehicle position->
		vehicleModel->setPosition(vector3df(LonToX(lon), veh.height*0.5, LatToZ(lat)));
		vehicleModel->setRotation(vector3df(0, -(90.0 - head), 0));
		mainCam->setPosition(vehicleModel->getPosition() + getPositionOnSphere(m_Rot.Y, m_Rot.X, m_Rad));
		mainCam->setTarget(vehicleModel->getPosition());

		driver->beginScene(true, true, SColor(255, 100, 101, 140));
		smgr->drawAll();
		guienv->drawAll();

		// Draw lines at vehicle position->
		SMaterial m;
		m.Lighting = false;
		driver->setMaterial(m);
		driver->setTransform(video::ETS_WORLD, core::matrix4());
		driver->draw3DLine(
			vehicleModel->getPosition() + core::vector3df(-5.0f, 0.0f, 0.0f),
			vehicleModel->getPosition() + core::vector3df(5.0f, 0.0f, 0.0f),
			SColor(255, 0, 100, 255)
		);
		driver->draw3DLine(
			vehicleModel->getPosition() + core::vector3df(0.0f, 0.0f, -5.0f),
			vehicleModel->getPosition() + core::vector3df(0.0f, 0.0f, 5.0f),
			SColor(255, 0, 100, 255)
		);

		// Draw a line that displays the vehicle's current heading.
		driver->draw3DLine(
			vehicleModel->getPosition(),
			vehicleModel->getPosition() + core::vector3df(sin(PI / 180.0 * head)*5.0*GetVehicle()->wheelSpeedN, 0.0f, GetVehicle()->wheelSpeedN*cos(PI / 180.0 * head)*5.0),
			SColor(255, 0, 255, 255)
		);
		// Draw a line that displays the vehicle's current GPS based heading.
		driver->draw3DLine(
			vehicleModel->getPosition(),
			vehicleModel->getPosition() + core::vector3df(sin(PI / 180.0 * GPSheading)*5.0, 0.0f, cos(PI / 180.0 * GPSheading)*5.0),
			SColor(255, 50, 255, 100)
		);

		// GPS error
		double dx = -(veh.gps.coords.lon - GPScoords.lon) * lonToM;
		double dz = -(veh.gps.coords.lat - GPScoords.lat) * latToM;
		driver->draw3DLine(
			vehicleModel->getPosition(),
			vehicleModel->getPosition() + vector3df(dx, 0, dz),
			SColor(255, 50, 255, 0)
		);
		/*
		for(int i = 0; i < payloads.size(); i++){
			driver->draw3DLine(
				vehicleModel->getPosition() + vector3df(dx, 0, dz),
				vehicleModel->getPosition() + payloads[i]->getPosition(),
				SColor(255, 50, 255, 0)
			);
		}
		*/
		error->setPosition(vehicleModel->getPosition() - vector3df(0, vehicleModel->getPosition().Y * 0.9, 0));

		// Check lasers (and draw lines for lasers in 3d space)
		for (int i = 0; i < veh.lasers.size(); i++) {
			vector3df laserPos = vehicleModel->getPosition() +
				vector3df(sin(PI / 180.0 * head)*veh.lasers[i].relativeForward*0.5, veh.lasers[i].relativeUpward*0.5, cos(PI / 180.0 * head)*veh.lasers[i].relativeForward*0.5) +
				vector3df(sin(PI / 180.0 * (head + 90))*veh.lasers[i].relativeRight*0.5, 0.0, cos(PI / 180.0 * (head + 90))*veh.lasers[i].relativeRight*0.5);
			vector3df laserEnd = laserPos + vector3df(sin(PI / 180.0 * head) * veh.lasers[i].detectionDistance, 0.0, cos(PI / 180.0 * head) * veh.lasers[i].detectionDistance);
			line3d<f32> ray;
			ray.start = laserPos;
			ray.end = laserEnd;
			vector3df intersection;
			triangle3df hitTriangle;
			ISceneCollisionManager* collMan = smgr->getSceneCollisionManager();
			ISceneNode* selectedNode = collMan->getSceneNodeAndCollisionPointFromRay(ray, intersection, hitTriangle, 0);

			if (selectedNode && selectedNode->getID() != vehicleModel->getID())
			{
				veh.lasers[i].val = true;
				driver->draw3DLine(laserPos, laserEnd, SColor(255, 255, 255, 0));
			}
			else {
				veh.lasers[i].val = false;
				driver->draw3DLine(laserPos, laserEnd, SColor(255, 255, 0, 0));
			}
		}

		// Draw main axes lines on the screen->
		driver->draw3DLine(
			core::vector3df(-1000.0f, 0.1f, 0.0f),
			core::vector3df(1000.0f, 0.1f, 0.0f),
			SColor(255, 255, 255, 255)
		);
		driver->draw3DLine(
			core::vector3df(0.0f, 0.1f, -1000.0f),
			core::vector3df(0.0f, 0.1f, 1000.0f),
			video::SColor(255, 255, 255, 255)
		);

		// Push the scene to the render buffer.
		driver->endScene();
	}
	else {
		device->drop();
		deviceDropped = true;
		// Check lasers (and draw lines for lasers in 3d space)
		for (int i = 0; i < veh.lasers.size(); i++) {
			veh.lasers[i].val = false;
		}
	}
}

vector3df PlantModel::getPositionOnSphere(f32 angleH, f32 angleV, f32 radius)
{
	// Get position on Z/Y Plane using conversion from polar
	// to cartesian coordinates
	f32 posZ = radius * cos(angleV);
	f32 posY = radius * sin(angleV);

	// Create a positional vector with X=0
	vector3df camPos(0, posY, posZ);

	// Create a transformation matrix to rotate the vector 'camPos'
	// around the Y Axis
	matrix4 yawMat;
	yawMat.setRotationRadians(vector3df(0, angleH, 0));
	yawMat.transformVect(camPos);

	return camPos;
}

bool PlantModel::OnEvent(const SEvent &event)
{
	if (event.EventType == EET_KEY_INPUT_EVENT)
	{
		const SEvent::SKeyInput *ev = &event.KeyInput;
		if (ev->Key == KEY_LEFT)
			m_Rot.Y -= 0.1f;
		else if (ev->Key == KEY_RIGHT)
			m_Rot.Y += 0.1f;
		else if (ev->Key == KEY_UP)
			m_Rad = (m_Rad > 3.0) ? 0.75*m_Rad : m_Rad;
		else if (ev->Key == KEY_DOWN)
			m_Rad = (m_Rad < 50.0) ? 1.5*m_Rad : m_Rad;

		return true;
	}
	else if (event.EventType == EET_MOUSE_INPUT_EVENT)
	{
		const SEvent::SMouseInput *ev = &event.MouseInput;
		if (ev->Event == EMIE_MOUSE_WHEEL)
		{
			if (ev->Wheel >= 0)
				m_Rad = (m_Rad > 3.0) ? 0.75*m_Rad : m_Rad;
			else
				m_Rad = (m_Rad < 50.0) ? 1.5*m_Rad : m_Rad;
		}
		else
		{
			if (!m_Dragging && ev->isLeftPressed())
			{
				m_DragStart.X = ev->X;
				m_DragStart.Y = ev->Y;
				m_DragStartRotation.X = m_Rot.X;
				m_DragStartRotation.Y = m_Rot.Y;
				m_Dragging = true;
			}
			else if (m_Dragging && !ev->isLeftPressed())
			{
				m_Dragging = false;
			}
			else if (m_Dragging && ev->isLeftPressed())
			{
				// Calculate a rotational offset in the range of -PI to +PI
				f32 dx = ((ev->X - m_DragStart.X) / driver->getScreenSize().Width) * PI;
				f32 dy = ((ev->Y - m_DragStart.Y) / driver->getScreenSize().Height) * PI;

				// Calculate the new total rotation
				m_Rot.X = m_DragStartRotation.X + dy;
				m_Rot.Y = m_DragStartRotation.Y + dx;
			}
		}
	}

	return false;
}
#endif

void PlantModel::PrintStatus() {
	std::cout << "t: " << GetElapsedSeconds() <<
		" --- lat: " << std::to_string(GetVehicle()->gps.coords.lat) << ", lon: " << GetVehicle()->gps.coords.lon << "\n";
}
std::chrono::duration<double> PlantModel::GetSimDuration() {
	return std::chrono::system_clock::now() - initTime;
}
double PlantModel::GetElapsedSeconds() {
	std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - initTime;
	return elapsed_seconds.count();
}
Vehicle * PlantModel::GetVehicle() {
	return &veh;
}
double PlantModel::LatToZ(double d) { return (d - initLat)*latToM; }
double PlantModel::LonToX(double d) { return (d - initLon)*lonToM; }
double PlantModel::XToLon(double d) { return (d) / lonToM + initLon; }
double PlantModel::ZToLat(double d) { return (d) / latToM + initLat; }
