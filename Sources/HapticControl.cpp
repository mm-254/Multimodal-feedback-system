
// Note: if textfile output is required, input '>output.txt && type output.txt' in Properties>Debugging>Command Line Arguments

////////////DECLARE MODALITIES//////////////////////////////////////////////////

// 1): Haptic Device
// 2): Vibrotactors
// 3): Visual Feedback
// 4): Peltier Element

int modality_weight;
int modality_LL;
int modality_temp;

//////////////////////////////////////////////////////////////////////////////

using namespace std;

#include <iostream>
#include <thread>
#include <sstream>

//JACO

#ifndef UNICODE
#define UNICODE
#endif
#include "KinovaTypes.h"
#ifdef _WIN32
#include <Windows.h>
#include "CommunicationLayer.h"
#include "CommandLayer.h"
#include <conio.h>
#endif

//A handle to the API.
#ifdef _WIN32
HINSTANCE commandLayer_handle;
#endif

//Function pointers to the functions we need
int(*MyInitAPI)();
int(*MyCloseAPI)();
int(*MySendBasicTrajectory)(TrajectoryPoint command);
int(*MyGetCartesianForce)(CartesianPosition &);
int(*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
int(*MySetActiveDevice)(KinovaDevice device);
int(*MyMoveHome)();
int(*MyInitFingers)();
int(*MyGetCartesianCommand)(CartesianPosition &);
int(*MyGetAngularForce)(AngularPosition &);
int(*MyGetAngularForceGravityFree)(AngularPosition &);
int(*MyGetSensorsInfo)(SensorsInfo &);
int(*MyGetAngularPosition)(AngularPosition &);

//Tactors

#include "tactor_cHeader.h"

string portName = "\\\\.\\COM01";
bool vibOn = true;
//vibration flag
int freq;
HANDLE Handle_Of_Vib_Thread = 0;

void init_Tactor();
// vibration thread
DWORD WINAPI Vib_Thread(LPVOID lpParam);

//this function vibrates a tactor
void vibrate(int tactor, int freq);

//Sensor Feedback

//sensor feedback thread
DWORD WINAPI feedbackThread(LPVOID lpParam);
HANDLE Handle_Of_Feedback_Thread;

//Serial communication (Sensors, Arduino)
#include "Serial.h"
#include <string.h>
#include <stdio.h>
#include <vector>
//connected to COM 8
ArdSerial tempSenseSerial(L"COM8");

//sense temperature and convert
float tempSense(void);

//liquid level
ArdSerial liquidLevSerial(L"COM9");
bool liquidLevSense(void);

//Peltier Device
DWORD WINAPI updatePelt(LPVOID lpParam);
HANDLE Handle_Of_Peltier;

ArdSerial peltWriteSerial(L"COM2");

//VD
DWORD WINAPI updateVD(LPVOID lpParam);
HANDLE Handle_Of_VD;

//Chai3D

#include "chai3d.h"
#include <GLFW/glfw3.h>
#include <CDeltaDevices.h>

using namespace chai3d;


//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------

cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;


//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cDirectionalLight *light;

// a small sphere (cursor) representing the haptic device 
cShapeSphere* cursor;

// a line representing the velocity vector of the haptic device
cShapeLine* velocity;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;

// a label to display the haptic device model
cLabel* labelHapticDeviceModel;

// a label to display the position [m] of the haptic device
cLabel* labelHapticDevicePosition;

// a global variable to store the position [m] of the haptic device
cVector3d hapticDevicePosition;

// a font for rendering text
cFontPtr font;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelRates;

// a flag for using damping (ON/OFF)
bool useDamping = false;

// a flag for using force field (ON/OFF)
bool useForceField = true;

// a flag to indicate if the haptic simulation currently running
bool simulationRunning = false;

// a flag to indicate if the haptic simulation has terminated
bool simulationFinished = true;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

// a handle to window display context
GLFWwindow* window = NULL;

// current width of window
int width = 0;
int width2 = 0;

// current height of window
int height = 0;
int height2 = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;

// haptic thread
cThread* hapticsThread;
//robot thread
cThread* robThread;

//weight feedback thread
cThread* weightThread;

//VD thread
// a label to describe application
cLabel* label_PropertyVal;
cLabel* label_PropertyName;

//Global variables for button
bool prebuttonStatus = false;
int buttonCount = 0;
bool button0 = false;

//Global feedback variables
bool weight = false;
bool LL = false;
bool temp = false;
float W_obj; // preset weight of the held object
float temperature; // temperature of object sensor points to
bool liquid_lev; // whether liquid level is upto sensor or not

//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback when the window display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);
void window2SizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// this function closes the application
void close(void);

// this function renders the scene
void updateGraphics(void);

// this function contains the main haptics simulation loop
void updateHaptics(void);

//this function computes and sends forces to the device
void sendHapticForce(cVector3d force, cVector3d torque, cVector3d desiredPosition, cMatrix3d desiredRotation);

//this function contains the robot update thread
void updateJACO(void);

//this function initializes the robot
void init_JACO(void);

//this function sends a point to the robot
void sendToJACO(float x, float y, float z, float tx, float ty, float tz, float f1, float f2, float f3);

//this function updates grasp feedback
//void updateVD(void);



//------------------------------------------------------------------------------
// MAIN LOOP
//------------------------------------------------------------------------------

int main(int argc, char* argv[])
{	//Chai3D

	//--------------------------------------------------------------------------
	// INITIALIZATION
	//--------------------------------------------------------------------------

	cout << endl;
	cout << "----------------------------------------------------------" << endl;
	cout << "Multimodal Feedback Interface" << endl;
	cout << "Mandira Marambe" << endl;
	cout << "Duerstock IAS Lab, Purdue University" << endl;
	cout << "----------------------------------------------------------" << endl << endl << endl;
	cout << "Keyboard Options:" << endl << endl;
	cout << "[q] - Exit application" << endl;
	cout << "[w] - Toggle on weight" << endl;
	cout << "[t] - Toggle on temperature" << endl;
	cout << "[l] - Toggle on liquid level" << endl;
	cout << "[s] - Toggle off all feedback" << endl;
	cout << endl << endl;

	cout << "----------------------------------------------------------- " << endl;
	cout << "Modalities: " << endl;
	cout << " 1: Haptic Device  2: Vibrotactors  3: Visual Display  4: Peltier Element " << endl;
	cout << "ENTER WEIGHT MODALITY: " << endl;
	cin >> modality_weight;
	cout << "Weight modality is: " << modality_weight << endl;
	cout << "ENTER TEMPERATURE MODALITY: " << endl;
	cin >> modality_temp;
	cout << "Temperature modality is: " << modality_temp << endl;
	cout << "ENTER LIQUID LEVEL MODALITY: " << endl;
	cin >> modality_LL;
	cout << "Liquid level modality is: " << modality_LL << endl;
	cout << "----------------------------------------------------------- " << endl;

	cout << "----------------------------------------------------------- " << endl;
	cout << "ENTER WEIGHT: " << endl;
	cin >> W_obj;
	cout << "Thank you. " << endl;
	cout << "----------------------------------------------------------- " << endl;
	
	//--------------------------------------------------------------------------
	// OPENGL - WINDOW DISPLAY
	//--------------------------------------------------------------------------

	// initialize GLFW library
	if (!glfwInit())
	{
		cout << "failed initialization" << endl;
		cSleepMs(1000);
		return 1;
	}

	// set error callback
	glfwSetErrorCallback(errorCallback);

	// compute desired size of window
	const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
	int w = 0.8 * mode->height;
	int h = 0.5 * mode->height;
	int x = 0.5 * (mode->width - w);
	int y = 0.5 * (mode->height - h);

	// set OpenGL version
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

	// set active stereo mode
	if (stereoMode == C_STEREO_ACTIVE)
	{
		glfwWindowHint(GLFW_STEREO, GL_TRUE);
	}
	else
	{
		glfwWindowHint(GLFW_STEREO, GL_FALSE);
	}

	// create display context
	window = glfwCreateWindow(w, h, "CHAI3D", NULL, NULL);
	if (!window)
	{
		cout << "failed to create window" << endl;
		cSleepMs(1000);
		glfwTerminate();
		return 1;
	}


	// get width and height of window
	glfwGetWindowSize(window, &width, &height);

	// set position of window
	glfwSetWindowPos(window, x, y);

	// set key callback
	glfwSetKeyCallback(window, keyCallback);

	// set resize callback
	glfwSetWindowSizeCallback(window, windowSizeCallback);

	// set current display context
	glfwMakeContextCurrent(window);

	// sets the swap interval for the current display context
	glfwSwapInterval(swapInterval);

#ifdef GLEW_VERSION
	// initialize GLEW library
	if (glewInit() != GLEW_OK)
	{
		cout << "failed to initialize GLEW library" << endl;
		glfwTerminate();
		return 1;
	}
#endif


	//--------------------------------------------------------------------------
	// WORLD - CAMERA - LIGHTING
	//--------------------------------------------------------------------------

	// create a new world.
	world = new cWorld();

	// set the background color of the environment
	world->m_backgroundColor.setBlack();

	// create a camera and insert it into the virtual world
	camera = new cCamera(world);
	world->addChild(camera);

	// position and orient the camera
	camera->set(cVector3d(0.5, 0.0, 0.0),    // camera position (eye)
		cVector3d(0.0, 0.0, 0.0),    // look at position (target)
		cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

									 // set the near and far clipping planes of the camera
	camera->setClippingPlanes(0.01, 10.0);

	// set stereo mode
	camera->setStereoMode(stereoMode);

	// set stereo eye separation and focal length (applies only if stereo is enabled)
	camera->setStereoEyeSeparation(0.01);
	camera->setStereoFocalLength(0.5);

	// set vertical mirrored display mode
	camera->setMirrorVertical(mirroredDisplay);

	// create a directional light source
	light = new cDirectionalLight(world);

	// insert light source inside world
	world->addChild(light);

	// enable light source
	light->setEnabled(true);

	// define direction of light beam
	light->setDir(-1.0, 0.0, 0.0);

	// create a sphere (cursor) to represent the haptic device
	cursor = new cShapeSphere(0.01);

	// insert cursor inside world
	world->addChild(cursor);

	// create small line to illustrate the velocity of the haptic device
	velocity = new cShapeLine(cVector3d(0, 0, 0),
		cVector3d(0, 0, 0));

	// insert line inside world
	world->addChild(velocity);

	//--------------------------------------------------------------------------
	// HAPTIC DEVICE
	//--------------------------------------------------------------------------

	// create a haptic device handler
	handler = new cHapticDeviceHandler();

	// get a handle to the first haptic device
	handler->getDevice(hapticDevice, 0);

	// open a connection to haptic device
	hapticDevice->open();

	// calibrate device (if necessary)
	hapticDevice->calibrate();

	// retrieve information about the current haptic device
	cHapticDeviceInfo info = hapticDevice->getSpecifications();

	// display a reference frame if haptic device supports orientations
	if (info.m_sensedRotation == true)
	{
		// display reference frame
		cursor->setShowFrame(true);

		// set the size of the reference frame
		cursor->setFrameSize(0.05);
	}

	// if the device has a gripper, enable the gripper to simulate a user switch
	hapticDevice->setEnableGripperUserSwitch(true);

	//--------------------------------------------------------------------------
	// WIDGETS
	//--------------------------------------------------------------------------

	// create a font
	font = NEW_CFONTCALIBRI20();

	// create a label to display the haptic device model
	labelHapticDeviceModel = new cLabel(font);
	camera->m_frontLayer->addChild(labelHapticDeviceModel);
	labelHapticDeviceModel->setText(info.m_modelName);

	// create a label to display the position of haptic device
	labelHapticDevicePosition = new cLabel(font);
	camera->m_frontLayer->addChild(labelHapticDevicePosition);

	// create a label to display the haptic and graphic rate of the simulation
	labelRates = new cLabel(font);
	camera->m_frontLayer->addChild(labelRates);

	//--------------------------------------------------------------------------
	// JACO
	//--------------------------------------------------------------------------
	//int programResult = 0;
#ifdef _WIN32
	//We load the API.
	commandLayer_handle = LoadLibrary(L"CommandLayerWindows.dll");

	//We load the functions from the library
	MyInitAPI = (int(*)()) GetProcAddress(commandLayer_handle, "InitAPI");
	MyCloseAPI = (int(*)()) GetProcAddress(commandLayer_handle, "CloseAPI");
	MyMoveHome = (int(*)()) GetProcAddress(commandLayer_handle, "MoveHome");
	MyInitFingers = (int(*)()) GetProcAddress(commandLayer_handle, "InitFingers");
	MyGetDevices = (int(*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) GetProcAddress(commandLayer_handle, "GetDevices");
	MySetActiveDevice = (int(*)(KinovaDevice devices)) GetProcAddress(commandLayer_handle, "SetActiveDevice");
	MySendBasicTrajectory = (int(*)(TrajectoryPoint)) GetProcAddress(commandLayer_handle, "SendBasicTrajectory");
	MyGetCartesianCommand = (int(*)(CartesianPosition &)) GetProcAddress(commandLayer_handle, "GetCartesianCommand");
	MyGetCartesianForce = (int(*)(CartesianPosition &)) GetProcAddress(commandLayer_handle, "GetCartesianForce");
	MyGetAngularForce = (int(*)(AngularPosition &)) GetProcAddress(commandLayer_handle, "GetAngularForce");
	MyGetAngularForceGravityFree = (int(*)(AngularPosition &)) GetProcAddress(commandLayer_handle, "GetAngularForceGravityFree");
	MyGetSensorsInfo = (int(*)(SensorsInfo &)) GetProcAddress(commandLayer_handle, "GetSensorsInfo");
	MyGetAngularPosition = (int(*)(AngularPosition &)) GetProcAddress(commandLayer_handle, "GetAngularPosition");

#endif

	//Verify that all functions has been loaded correctly
	if ((MyInitAPI == NULL) || (MyCloseAPI == NULL) || (MySendBasicTrajectory == NULL) ||
		(MyGetDevices == NULL) || (MySetActiveDevice == NULL) || (MyGetCartesianCommand == NULL) || (MyGetCartesianForce == NULL) ||
		(MyMoveHome == NULL) || (MyInitFingers == NULL) || (MyGetSensorsInfo == NULL) || (MyGetAngularPosition == NULL))
	{
		cout << "* * *  E R R O R   D U R I N G   I N I T I A L I Z A T I O N  * * *" << endl;
		//programResult = 0;
	}
	else
	{
		cout << "----------------------------------------------------------- " << endl;
		cout << "R O B O T  I N I T I A L I Z A T I O N   C O M P L E T E D" << endl << endl;
		int result = (*MyInitAPI)();
		cout << "Initialization's result :" << result << endl;
		KinovaDevice list[MAX_KINOVA_DEVICE];
		int devicesCount = MyGetDevices(list, result);
		cout << "Found a robot on the USB bus (" << list[0].SerialNumber << ")" << endl;
		//Setting the current device as the active device.
		MySetActiveDevice(list[0]);
		cout << "Send the robot to HOME position" << endl;
		MyMoveHome();
		cout << "Initializing the fingers" << endl;
		MyInitFingers();
		cout << "----------------------------------------------------------- " << endl;
	}
	
	//--------------------------------------------------------------------------
	// START SIMULATION
	//--------------------------------------------------------------------------

	// create a thread which starts the main haptics rendering loop
	hapticsThread = new cThread();
	hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);	
	robThread = new cThread();
	robThread->start(updateJACO, CTHREAD_PRIORITY_HAPTICS);
	cout << "----------------------------------------------------------- " << endl;
	cout << "Started haptic and robot threads" << endl;

	// Vibration: Initialize Tactors 
	cout << "Initializing tactors" << endl;
	init_Tactor();
	cout << "Initialized tactors" << endl;
	//create a thread which starts the vibration response
	
	if (vibOn == TRUE) {
		int Data_Of_Vib_Thread = 1;
		Handle_Of_Vib_Thread = CreateThread(NULL, 0, Vib_Thread, &Data_Of_Vib_Thread, 0, NULL);
	}
	cout << "Started vibration thread" << endl;

	//Sensor feedback thread
	int Data_Of_Feedback_Thread = 1;
	Handle_Of_Feedback_Thread = CreateThread(NULL, 0, feedbackThread, &Data_Of_Feedback_Thread, 0, NULL);

	if (tempSenseSerial.IsConnected()) {
		cout << "Temperature sensor connected" << endl;
	}
	else {
		cout << "ERROR: Temperature sensor not connected" << endl;
	}

	if (liquidLevSerial.IsConnected()) {
		cout << "Liquid level sensor connected" << endl;
	}
	else {
		cout << "ERROR: Liquid level sensor not connected" << endl;
	}

	//Peltier thread
	int Data_Of_Peltier_Thread = 1;
	Handle_Of_Peltier = CreateThread(NULL, 0, updatePelt, &Data_Of_Peltier_Thread, 0, NULL);

	if (peltWriteSerial.IsConnected()) {
		cout << "Peltier device connected" << endl;
	}
	else {
		cout << "ERROR: Peltier device not connected" << endl;
	}
	cout << "----------------------------------------------------------- " << endl;
	//Visual feedback
	int Data_Of_VD_Thread = 1;
	Handle_Of_VD = CreateThread(NULL, 0, updateVD, &Data_Of_VD_Thread, 0, NULL);

	// setup callback when application exits
	atexit(close);
	//--------------------------------------------------------------------------
	// MAIN GRAPHIC LOOP
	//--------------------------------------------------------------------------
	// call window size callback at initialization
	windowSizeCallback(window, width, height);

	// main graphic loop
	while (!glfwWindowShouldClose(window))
	{
		// get width and height of window
		glfwGetWindowSize(window, &width, &height);

		// render graphics
		updateGraphics();

		// swap buffers
		glfwSwapBuffers(window);

		// process events
		glfwPollEvents();

		// signal frequency counter
		freqCounterGraphics.signal(1);

	}

	// close window
	glfwDestroyWindow(window);

	// terminate GLFW library
	glfwTerminate();



}


//==============================================================================
/*
FUNCTIONS

In the main haptics loop function  "updateHaptics()" , the position,
orientation and user switch status are read at each haptic cycle.
Force and torque vectors are computed and sent back to the haptic device.
*/
//==============================================================================


void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
	// update window size
	width = a_width;
	height = a_height;

	// update position of label
	labelHapticDeviceModel->setLocalPos(20, height - 40, 0);

	// update position of label
	labelHapticDevicePosition->setLocalPos(20, height - 60, 0);
	
}

//------------------------------------------------------------------------------

void errorCallback(int a_error, const char* a_description)
{
	cout << "Error: " << a_description << endl;
}

//------------------------------------------------------------------------------

void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
	// filter calls that only include a key press
	if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT))
	{
		return;
	}

	// option - exit
	else if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
	{
		glfwSetWindowShouldClose(a_window, GLFW_TRUE);
	}

	// option - enable/disable force field
	else if (a_key == GLFW_KEY_1)
	{
		useForceField = !useForceField;
		if (useForceField)
			cout << "> Enable force field     \r";
		else
			cout << "> Disable force field    \r";
	}

	// option - enable/disable damping
	else if (a_key == GLFW_KEY_2)
	{
		useDamping = !useDamping;
		if (useDamping)
			cout << "> Enable damping         \r";
		else
			cout << "> Disable damping        \r";
	}

	// option - toggle fullscreen
	else if (a_key == GLFW_KEY_F)
	{
		// toggle state variable
		fullscreen = !fullscreen;

		// get handle to monitor
		GLFWmonitor* monitor = glfwGetPrimaryMonitor();

		// get information about monitor
		const GLFWvidmode* mode = glfwGetVideoMode(monitor);

		// set fullscreen or window mode
		if (fullscreen)
		{
			glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
			glfwSwapInterval(swapInterval);
		}
		else
		{
			int w = 0.8 * mode->height;
			int h = 0.5 * mode->height;
			int x = 0.5 * (mode->width - w);
			int y = 0.5 * (mode->height - h);
			glfwSetWindowMonitor(window, NULL, x, y, w, h, mode->refreshRate);
			glfwSwapInterval(swapInterval);
		}
	}

	// option - toggle vertical mirroring
	else if (a_key == GLFW_KEY_M)
	{
		mirroredDisplay = !mirroredDisplay;
		camera->setMirrorVertical(mirroredDisplay);
	}
	else if (a_key == GLFW_KEY_W)
	{
		weight = true;
		cout << "weight on" << endl;
	}
	else if (a_key == GLFW_KEY_T)
	{
		temp = true;
		cout << "temp on" << endl;
		
	}
	else if (a_key == GLFW_KEY_L)
	{
		LL = true;
		cout << "liquid level on" << endl;


	}
	else if (a_key == GLFW_KEY_S)
	{
		weight = false;
		temp = false;
		LL = false;
	}
}

//------------------------------------------------------------------------------

void close(void)
{
	// stop the simulation
	simulationRunning = false;

	// wait for graphics and haptics loops to terminate
	while (!simulationFinished) { cSleepMs(100); }

	// close haptic device
	hapticDevice->close();

	// close vibration thread
	if (vibOn == TRUE)
		CloseHandle(Handle_Of_Vib_Thread);
		KillDLL();

	//close feedback thread
		CloseHandle(Handle_Of_Feedback_Thread);
	//close peltier thread
		CloseHandle(Handle_Of_Peltier);
	//close visual display thread
		CloseHandle(Handle_Of_VD);

	// delete resources
	delete hapticsThread;
	delete weightThread;
	//delete feedbackThread;
	delete robThread;
	delete world;
	delete handler;

	cout << "Send the robot to HOME position" << endl;
	MyMoveHome();
	cout << endl << "C L O S I N G   A P I" << endl;
	Sleep(100);
	int result = (*MyCloseAPI)();

#ifdef __linux__ 
	dlclose(commandLayer_handle);
#elif _WIN32
	FreeLibrary(commandLayer_handle);
#endif

}

//------------------------------------------------------------------------------

void updateGraphics(void)
{
	/////////////////////////////////////////////////////////////////////
	// UPDATE WIDGETS
	/////////////////////////////////////////////////////////////////////

	// update position data
	labelHapticDevicePosition->setText(hapticDevicePosition.str(3));

	// update haptic and graphic rate data
	labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
		cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");

	// update position of label
	labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);


	/////////////////////////////////////////////////////////////////////
	// RENDER SCENE
	/////////////////////////////////////////////////////////////////////

	// update shadow maps (if any)
	world->updateShadowMaps(false, mirroredDisplay);

	// render world
	camera->renderView(width, height);

	// wait until all OpenGL commands are completed
	glFinish();

	// check for any OpenGL errors
	GLenum err;
	err = glGetError();
	if (err != GL_NO_ERROR) cout << "Error:  %s\n" << gluErrorString(err);
}

//---------------------------------------------------------------------------------------------

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                              ROBOT CONTROL                                                      //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void updateJACO(void) {

	//update JACO based on haptic device position and joystick mode
	// simulation in now running
	simulationRunning = true;
	simulationFinished = false;
	int translation_axis;

	// main haptic simulation loop
	while (simulationRunning) {

		/////////////////////////////////////////////////////////////////////
		// UPDATE JACO
		/////////////////////////////////////////////////////////////////////
		//JACO joint variables
		float x; float y; float z; //translations
		float tx; float ty; float tz; //rotations
		float f1; float f2; float f3; //fingers
		// read user-switch status (button 0)

		hapticDevice->getUserSwitch(0, button0);  //make this keyboard switchable

		if (button0)
		{
			//cout << "Button Pressed" << endl;

			if (prebuttonStatus == 0) {
				//cout << prebuttonStatus << endl;
				buttonCount = buttonCount+1;
				prebuttonStatus = button0;
			}

			//Read haptic device position, orientation
			cVector3d position;
			cVector3d angle;
			hapticDevice->getPosition(position); // read position 
			hapticDevice->getAngularVelocity(angle); // read rotation

			//cout << buttonCount << endl;

			switch (buttonCount) {

			case 1:
				//Translations
				if (abs(position.get(1)) > 0.01) {
					translation_axis = 1;
					x = -1.2*position.get(1);
					cout << "Axis 01: " << abs(position.get(1))<<endl;
				}
				else if (abs(position.get(0)) > 0.01) {
					translation_axis = 2;
					y = 1.2*position.get(0);
					cout << "Axis 02: " << abs(position.get(0))<<endl;
				}
				else if (abs(position.get(2)) > 0.02 ) {
					translation_axis = 3;
					z = 1.2*position.get(2);
					cout << "Axis 03: " << abs(position.get(2))<< endl;
				}
				else {
					translation_axis = 0;
					x = 0; y = 0; z = 0;
				}
				switch (translation_axis) {
	
				case 1:
					x = -1.2*position.get(1);
					//sendToJACO(x, 0, 0, 0, 0, 0, 0, 0, 0);
					break;

				case 2:
					y = 1.2*position.get(0);
					//sendToJACO(0, y, 0, 0, 0, 0, 0, 0, 0);
					break;

				case 3:
					z = 1.2*position.get(2);
					//sendToJACO(0, 0, z, 0, 0, 0, 0, 0, 0);
					break;

				default:
					x = 0; y = 0; z = 0;
					break;
				}
				//else {
					//sendToJACO(0, 0, 0, 0, 0, 0, 0, 0, 0);
				//}

				sendToJACO(x, y, z, 0, 0, 0, 0, 0, 0);
				break;

			case 2:
				//Rotations
				tx = -angle.get(1);
				ty = angle.get(0);
				tz = 0;
				sendToJACO(0, 0, 0, tx, ty, tz, 0, 0, 0);
				break;

			case 3:
				//Wrist
				tz = -3 * angle.get(0);

				//Gripper
				if (abs(position.get(0)) > 0.02) {
					f1 = 1000 * position.get(0);
					f2 = 1000 * position.get(0);
					f3 = 1000 * position.get(0);
					cout << "Opening/Closing Gripper." << endl;
				}

				sendToJACO(0, 0, 0, 0, 0, tz, f1, f2, f3);
				break;
				
			default:
				buttonCount = 0;
			}
		}
		else
		{
			//Off
			prebuttonStatus = button0;
			sendToJACO(0, 0, 0, 0, 0, 0, 0, 0, 0);			
			weight = false;
			LL = false;
			temp = false;
		}
	}
	// exit thread
	simulationFinished = true;
}
//-----------------------------------------------------------------------------------------------------------------

void init_JACO(void) {



}
//-----------------------------------------------------------------------------------------------------------------

void sendToJACO(float x, float y, float z, float tx, float ty, float tz, float f1, float f2, float f3) {

	TrajectoryPoint pointToSend;
	pointToSend.InitStruct();

	//We specify that this point will be an angular(joint by joint) position.
	pointToSend.Position.Type = CARTESIAN_VELOCITY;
	pointToSend.Position.HandMode = VELOCITY_MODE;

	pointToSend.Position.CartesianPosition.X = x;
	pointToSend.Position.CartesianPosition.Y = y;
	pointToSend.Position.CartesianPosition.Z = z;
	pointToSend.Position.CartesianPosition.ThetaX = tx;
	pointToSend.Position.CartesianPosition.ThetaY = ty;
	pointToSend.Position.CartesianPosition.ThetaZ = tz;
	pointToSend.Position.Fingers.Finger1 = f1;
	pointToSend.Position.Fingers.Finger2 = f2;
	pointToSend.Position.Fingers.Finger3 = f3;

	for (int i = 0; i < 100; i++)
	{
		//send the velocity vector every 1 ms as long as we want the robot to move along that vector.
		MySendBasicTrajectory(pointToSend);
		//cout << "*********************************" << endl;
		Sleep(5);
	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                              SENSOR FUNCTIONS                                                  //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

DWORD WINAPI feedbackThread(LPVOID lpParam) {
	//This thread will monitor sensor signals 
	// simulation in now running

	simulationRunning = true;
	simulationFinished = false;
	
	float g = -9.81; // gravitational acceleration
	float m_hand = 0.93; // mass of the hand in kg 
	CartesianPosition prev_forces;
	cVector3d force;
	cVector3d torque;
	


	while (simulationRunning) {

		CartesianPosition forces;
		SensorsInfo data; //stores acceleration and temperature information
		AngularPosition angles; //stores angular position information
		CartesianPosition positions;//stores cartesian and angular positions
				
		//get force vector
		(*MyGetCartesianForce)(forces);
		//get angular position vector
		(*MyGetAngularPosition)(angles);
		//get sensor information vector
		(*MyGetSensorsInfo)(data);
		(*MyGetCartesianCommand)(positions);

		/////////////////////////////////////////////////////////////////////
		// WEIGHT
		/////////////////////////////////////////////////////////////////////

		//Sense Weight
		//Fz = forces.Coordinates.Z;
		//a_z = data.AccelerationZ;

		//W_obj = W_obj = ((Fz / a_z) - m_hand)*g;

		/////////////////////////////////////////////////////////////////////
		// LIQUID LEVEL
		/////////////////////////////////////////////////////////////////////
		liquid_lev = liquidLevSense();
		//cout<< "  Liquid level : " << liquid_lev << endl;

		/////////////////////////////////////////////////////////////////////
		// TEMPERATURE
		/////////////////////////////////////////////////////////////////////

		//Sense Temp

		temperature = tempSense();
		//cout << "  Temperature : " << temperature << endl;
		
	}
	// exit thread
	simulationFinished = true;
	return(0);
}
//--------------------------------------------------------------------------------------------------------------------

float tempSense(void) {
	const int RLEN = 22;
	char creadBuffer[RLEN + 1];
	int bytes = 0;
	memset(creadBuffer, '\0', RLEN + 1);
	Sleep(100);
	

	// Read data from the COM-port
	bytes = tempSenseSerial.ReadData(creadBuffer, 14);

	if (bytes ==14)
	{
		// Finalize the data, so it is a valid string
		//creadBuffer[bytes/2] = '\0';
		creadBuffer[bytes] = '\0';
		string str(creadBuffer);
		//separate object and ambient temperatures
		string obj = str.substr(0,5);
		string amb = str.substr(7, 11);

		float objtemp = stof(obj); //convert object temp to float
		float ambtemp = stof(amb); //convert ambient temp to float

		if (ambtemp>50 && objtemp > 10) { //filtering out incomplete data bytes; Assuming ambient temps would be over 50. Adjust this value if expecting to operate in outdoor cold weather
			//cout << "Object:" << objtemp << endl;
			//cout << "Ambient:" << ambtemp << endl;
			//cout << "Diff:" << ambtemp - objtemp << endl;
			//cout << " " << endl;

			return objtemp;
		}
		else {
			return temperature;
		}

	}
	else {
		return temperature;
	}

}
//--------------------------------------------------------------------------------------------------------------

bool liquidLevSense(void) {
	const int RLEN = 3;
	char creadBuffer[RLEN + 1];
	int bytes = 0;
	memset(creadBuffer, '\0', RLEN + 1);
	Sleep(300);

	// Read data from the COM-port
	bytes = liquidLevSerial.ReadData(creadBuffer, 3);

	if (bytes > 0)
	{
		// Finalize the data, so it is a valid string
		int lev = atoi(creadBuffer);
		if (lev==1) {
			return 1;
		}
		if (lev==0) {
			return 0;
		}
	}
	else {
		return liquid_lev;
	}

}

//---------------------------------------------------------------------------------------------------------------------

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                 FEEDBACK FUNCTIONS                                                            //
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Feedback functions are divided into properties, then within functions, modalities are identified through integers where:
// 01) Haptic feedback [updateHaptics() function]
// 02) Vibration [updateVib() function]
// 03) Visual display [updateVD() function]
// 04) Peltier element [updatePelt() function]

void updateHaptics(void)
{
	//note: default update haptics state will be to send the device to a desired position of (0,0,0)

	// simulation in now running
	simulationRunning = true;
	simulationFinished = false;
	// main haptic simulation loop
	while (simulationRunning)
	{
		/////////////////////////////////////////////////////////////////////
		// READ HAPTIC DEVICE
		/////////////////////////////////////////////////////////////////////

		// read position 
		cVector3d position;
		hapticDevice->getPosition(position);

		// read orientation 
		cMatrix3d rotation;
		hapticDevice->getRotation(rotation);

		// read linear velocity 
		cVector3d linearVelocity;
		hapticDevice->getLinearVelocity(linearVelocity);

		// read user-switch status (button 0)
		bool button0;
		button0 = false;
		hapticDevice->getUserSwitch(0, button0);


		/////////////////////////////////////////////////////////////////////
		// UPDATE cursor
		/////////////////////////////////////////////////////////////////////

		// update arrow
		velocity->m_pointA = position;
		velocity->m_pointB = cAdd(position, linearVelocity);

		// update position and orientation of cursor
		cursor->setLocalPos(position);
		cursor->setLocalRot(rotation);

		// adjust the  color of the cursor according to the status of
		// the user-switch (ON = TRUE / OFF = FALSE)
		if (button0)
		{
			cursor->m_material->setGreenMediumAquamarine();
		}
		else
		{
			cursor->m_material->setBlueRoyal();
		}

		// update global variable for graphic display update
		hapticDevicePosition = position;


		/////////////////////////////////////////////////////////////////////
		// APPLY FORCES
		/////////////////////////////////////////////////////////////////////

		// desired position
		cVector3d desiredPosition;
		desiredPosition.set(0.0, 0.0, 0.0);

		// desired orientation
		cMatrix3d desiredRotation;
		desiredRotation.identity();

		// variables for forces
		cVector3d force(0, 0, 0);
		cVector3d torque(0, 0, 0);

		if (weight == true && modality_weight == 1) {

			///////////////Haptic weight feedback////////////////////////////////
			force(2) = -W_obj; //this number will have to be from sensor readings
			desiredPosition.set(0.0, 0.0, position(2)); //prevent device from humping back to 0
			sendHapticForce(force, torque, desiredPosition, desiredRotation); // compute and send force, torque, and gripper force to haptic device

		}
		else if (LL == true && modality_LL == 1) {

			///////////////Haptic liquid level feedback////////////////////////////////
			if (liquid_lev == 1) {
				force(2) = -2; //this number will have to be from sensor readings
				desiredPosition.set(0.0, 0.0, position(2)); //prevent device from humping back to 0
				sendHapticForce(force, torque, desiredPosition, desiredRotation); // compute and send force, torque, and gripper force to haptic device
			}

		}
		else { //default
			force = (0, 0, 0);
			torque = (0, 0, 0); 
			desiredPosition = (0, 0, 0);
			sendHapticForce(force, torque, desiredPosition, desiredRotation); // compute and send force, torque, and gripper force to haptic device
		}

		// signal frequency counter
		freqCounterHaptics.signal(1);
	}

	// exit haptics thread
	simulationFinished = true;
}
//--------------------------------------------------------------------------------------------------------------------

void sendHapticForce(cVector3d force, cVector3d torque, cVector3d desiredPosition, cMatrix3d desiredRotation) {

	/////////////////////////////////////////////////////////////////////
	// READ DEVICE, COMPUTE AND APPLY FORCES
	/////////////////////////////////////////////////////////////////////

	double gripperForce = 0.0;
	// read position 
	cVector3d position;
	hapticDevice->getPosition(position);
	// read orientation 
	cMatrix3d rotation;
	hapticDevice->getRotation(rotation);
	// read gripper position
	double gripperAngle;
	hapticDevice->getGripperAngleRad(gripperAngle);
	// read linear velocity 
	cVector3d linearVelocity;
	hapticDevice->getLinearVelocity(linearVelocity);
	// read angular velocity
	cVector3d angularVelocity;
	hapticDevice->getAngularVelocity(angularVelocity);
	// read gripper angular velocity
	double gripperAngularVelocity;
	hapticDevice->getGripperAngularVelocity(gripperAngularVelocity);

	// apply force field
	if (useForceField)
	{
		// compute linear force
		double Kp = 25; // [N/m]
		cVector3d forceField = Kp * (desiredPosition - position);
		force.add(forceField);

		// compute angular torque
		double Kr = 0.05; // [N/m.rad]
		cVector3d axis;
		double angle;
		cMatrix3d deltaRotation = cTranspose(rotation) * desiredRotation;
		deltaRotation.toAxisAngle(axis, angle);
		torque = rotation * ((Kr * angle) * axis);
	}

	// apply damping term
	if (useDamping)
	{
		cHapticDeviceInfo info = hapticDevice->getSpecifications();

		// compute linear damping force
		double Kv = 1.0 * info.m_maxLinearDamping;
		cVector3d forceDamping = -Kv * linearVelocity;
		force.add(forceDamping);

		// compute angular damping force
		double Kvr = 1.0 * info.m_maxAngularDamping;
		cVector3d torqueDamping = -Kvr * angularVelocity;
		torque.add(torqueDamping);

		// compute gripper angular damping force
		double Kvg = 1.0 * info.m_maxGripperAngularDamping;
		gripperForce = gripperForce - Kvg * gripperAngularVelocity;
	}

	// send computed force, torque, and gripper force to haptic device
	hapticDevice->setForceAndTorqueAndGripperForce(force, torque, gripperForce);
}

//---------------------------------------------------------------------------------------------

DWORD WINAPI Vib_Thread(LPVOID lpParam) {

	// simulation in now running

	simulationRunning = true;
	simulationFinished = false;

	while (simulationRunning == true) {
		
		if (weight == true && modality_weight == 2) {
			freq = (W_obj*319/10)+30;
			vibrate(0, freq);
		}
		else if (LL == true && modality_LL == 2) {
			if (liquid_lev == 1) {
				vibrate(0, 300);
			}
		}
		else if (temp == true && modality_temp == 2) { //assume temp range from 60F to 140F (0-60C)
			freq = ((temperature-60) * 319 / 80) + 30;
			vibrate(0, freq);
		}
		else {
			freq = 0;
		}
		
	}
	// exit thread
	simulationFinished = true;
	return(0);
}

//--------------------------------------------------------------------------------------------
void init_Tactor()
{
	if (InitTactorDLL() < 0) {
		cout << "ERROR DURING INIT"<<endl;
		//return -1;
	}

	cout << "beforeDicover"<<endl;
	int amountDiscovered = DiscoverDevices(USBADV | Serial | USBEVAL);
	cout << amountDiscovered<<endl;
	int returnval = Connect(0, USBADV | Serial | USBEVAL);

	//cout << portName.c_str();
	//int returnval = ConnectDirect((char*)portName.c_str(), USBADV|Serial|USBEVAL);
	if (returnval < 0) {
		cout << "Error Connecting to Controller" << returnval << endl;
		vibOn = FALSE;
		//return -1;
	}
	else {
		vibOn = TRUE;
		cout << "connected to tactor." << endl;
	}

}
//------------------------------------------------------------------------------------------------

void vibrate(int tactor, int freq)
{
		// The second parameter chooses which tactor, or use 0 to set all.
		// In our case, we want to set tactors 1, 2, and 3.
	//cout << freq << endl;
	SetSinFreq1(0, 0, freq, true);
	if (tactor == 1) {
		TaconTime(0, 0, Tac1, 150, true);
	}
	else if (tactor == 2) {
		TaconTime(0, 0, Tac2, 150, true);
	}
	else if (tactor == 3) {
		TaconTime(0, 0, Tac3, 150, true);
	}
	else if (tactor == 4) {
		TaconTime(0, 0, Tac4, 150, true);
	}
	else {
		TaconTime(0, 0, All, 150, true); //All tactors
	}

}
//---------------------------------------------------------------------------------------------

DWORD WINAPI updateVD(LPVOID lpParam) {

	//------------------------------------------------------------------------------
	// DECLARED VARIABLES
	//------------------------------------------------------------------------------

	// a world that contains all objects of the virtual environment
	cWorld* world2;
	// a camera to render the world in the window display
	cCamera* camera2;
	// a light source to illuminate the objects in the world
	cDirectionalLight *light2;

	// a font for rendering text
	cFontPtr font2;
	// a handle to window display context
	GLFWwindow* window2 = NULL;

	// swap interval for the display context (vertical synchronization)
	int swapInterval2 = 1;
	// simulation in now running

	//--------------------------------------------------------------------------
	// OPEN GL - WINDOW DISPLAY
	//--------------------------------------------------------------------------

	// initialize GLFW library
	if (!glfwInit())
	{
		cout << "failed initialization" << endl;
		cSleepMs(1000);
		return 1;
	}
	// set error callback
	glfwSetErrorCallback(errorCallback);

	// compute desired size of window
	const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
	int w2 = 0.8 * mode->height;
	int h2 = 0.5 * mode->height;
	int x2 = 0.5 * (mode->width - w2);
	int y2 = 0.5 * (mode->height - h2);

	// set OpenGL version
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

	// set active stereo mode
	if (stereoMode == C_STEREO_ACTIVE)
	{
		glfwWindowHint(GLFW_STEREO, GL_TRUE);
	}
	else
	{
		glfwWindowHint(GLFW_STEREO, GL_FALSE);
	}

	// create display context
	window2 = glfwCreateWindow(w2, h2, "VISUAL DISPLAY", NULL, NULL);
	if (!window2)
	{
		cout << "failed to create window" << endl;
		cSleepMs(1000);
		glfwTerminate();
		//return 1;
	}

	// get width and height of window
	glfwGetWindowSize(window2, &width2, &height2);

	// set position of window
	glfwSetWindowPos(window2, x2, y2);

	// set key callback
	glfwSetKeyCallback(window2, keyCallback);

	// set resize callback
	glfwSetWindowSizeCallback(window2, window2SizeCallback);

	// set current display context
	glfwMakeContextCurrent(window2);

	// sets the swap interval for the current display context
	glfwSwapInterval(swapInterval2);


	//--------------------------------------------------------------------------
	// WORLD - CAMERA - LIGHTING
	//--------------------------------------------------------------------------

	// create a new world.
	world2 = new cWorld();

	// set the background color of the environment

	world2->m_backgroundColor.set(1, 1, 1);

	// create a camera and insert it into the virtual world
	camera2 = new cCamera(world2);
	world2->addChild(camera2);

	// position and orient the camera
	camera2->set(cVector3d(3.0, 0.0, 0.0),    // camera position (eye)
		cVector3d(0.0, 0.0, 0.0),    // lookat position (target)
		cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

									 // set the near and far clipping planes of the camera
	camera2->setClippingPlanes(0.01, 10.0);

	// set stereo mode
	camera2->setStereoMode(stereoMode);

	// set stereo eye separation and focal length (applies only if stereo is enabled)
	camera2->setStereoEyeSeparation(0.02);
	camera2->setStereoFocalLength(3.0);

	// set vertical mirrored display mode
	camera2->setMirrorVertical(mirroredDisplay);

	// enable multi-pass rendering to handle transparent objects
	camera2->setUseMultipassTransparency(true);

	// create a directional light source
	light2 = new cDirectionalLight(world2);

	// insert light source inside world
	world2->addChild(light2);

	// enable light source
	light2->setEnabled(true);

	// define direction of light beam
	light2->setDir(-1.0, 0.0, 0.0);


	//--------------------------------------------------------------------------
	// WIDGETS
	//--------------------------------------------------------------------------

	font2 = NEW_CFONTCALIBRI20();

	// create a label that is updated with the number in update graphics
	label_PropertyVal = new cLabel(font2);
	camera2->m_frontLayer->addChild(label_PropertyVal);
	label_PropertyVal->m_fontColor.setBlack();

	// create a new label for the title
	label_PropertyName = new cLabel(font);
	camera2->m_frontLayer->addChild(label_PropertyName);
	label_PropertyName->m_fontColor.setBlack();

	// sets different title labels for different Modalities
	if (modality_temp == 3)
	{
		label_PropertyName->setText("TEMPERATURE (F)");
	}
	else if (modality_weight == 3)
	{
		label_PropertyName->setText("WEIGHT (lbs)");
	}
	else if (modality_LL == 3)
	{
		label_PropertyName->setText("   ");
	}
	label_PropertyName->setFontScale(4);

	// call window size callback at initialization
	window2SizeCallback(window2, width2, height2);


	simulationRunning = true;
	simulationFinished = false;

	while (!glfwWindowShouldClose(window2) && simulationRunning== true) {

		// get width and height of window
		glfwGetWindowSize(window2, &width2, &height2);

		// update position of label
		/////////////////////////////////////////////////////////////////////
		// UPDATE BITMAP IMAGES
		/////////////////////////////////////////////////////////////////////

		// creates a new label with a blank text in order to update to the next number without having an overlap
		double ratioMax = 0.0; // sets label ratio
		label_PropertyVal->setText("                      "); // sets blank spaces in order to acheive a clean canvas to prevent overlapping
		label_PropertyVal = new cLabel(font); // sets font settings to label
		camera2->m_frontLayer->addChild(label_PropertyVal); // adds a new blank label so the numbers do not overlap
														  // create new label object
		label_PropertyVal = new cLabel(font); // sets font to label
		camera2->m_frontLayer->addChild(label_PropertyVal); // adds a new label for number display


		string Result;          // string which will contain the result
		ostringstream convert;   // stream used for the conversion
									 // sets conditions for color change with change in temperature

		if (weight == true && modality_weight == 3) {
			W_obj = round(W_obj * 10) / 10; // rounds the update float to 1 decimal place
			float Number = W_obj;       // number to be converted to a string
			convert << Number;      // insert the textual representation of 'Number' in the characters in the stream
			Result = convert.str(); // set 'Result' to the contents of the stream
			label_PropertyVal->setText(Result); // sets the result as string to the label
			label_PropertyVal->setFontScale(4); // sets font size

			if (0 <= W_obj && W_obj < 3)
			{
				world2->m_backgroundColor.set(0.5 - (((W_obj) / 10) - 0.25), 1, 0.5 - (((W_obj) / 10) - 0.25));
				label_PropertyVal->m_fontColor.setBlack();
				label_PropertyName->m_fontColor.setBlack();
			}

			else if (3 <= W_obj && W_obj < 6)
			{
				world2->m_backgroundColor.set(1 - (((W_obj - 3) / 10) - 0.25), 1 - (((W_obj - 3) / 10) - 0.25), 0);
				label_PropertyVal->m_fontColor.setBlack();
				label_PropertyName->m_fontColor.setBlack();
			}

			else if (6 <= W_obj && W_obj <= 10)
			{
				world2->m_backgroundColor.set(1, 0.5 - (((W_obj - 5) / 10) - 0.25), 0.5 - (((W_obj - 5) / 10) - 0.25));
				label_PropertyVal->m_fontColor.setBlack();
				label_PropertyName->m_fontColor.setBlack();
			}

			else if (W_obj > 10)
			{
				world2->m_backgroundColor.set(0, 0, 0);
				label_PropertyVal->setText("Weight ABOVE device limit");
				label_PropertyVal->m_fontColor.setWhite();
				label_PropertyName->m_fontColor.setWhite();
			}

			else if (W_obj < 0)
			{
				world2->m_backgroundColor.set(0, 0, 0);
				label_PropertyVal->setText("Weight BELOW device limit");
				label_PropertyVal->m_fontColor.setWhite();
				label_PropertyName->m_fontColor.setWhite();
			}

		}
		else if (LL == true && modality_LL == 3) {
			liquid_lev = round(liquid_lev * 10) / 10; // rounds the update float to 1 decimal place
			float Number = liquid_lev;       // number to be converted to a string
			convert << Number;      // insert the textual representation of 'Number' in the characters in the stream
			Result = convert.str(); // set 'Result' to the contents of the stream
			label_PropertyVal->setText(Result); // sets the result as string to the label
			label_PropertyVal->setFontScale(4); // sets font size

			if (liquid_lev == false)
			{
				world2->m_backgroundColor.set(0, 0, 0);
				label_PropertyVal->setText("Liquid is BELOW level");
				label_PropertyVal->m_fontColor.setWhite();
				label_PropertyName->m_fontColor.setWhite();
			}
			else if (liquid_lev == true)
			{
				world2->m_backgroundColor.set(1, 1, 0);
				label_PropertyVal->setText("Liquid is ABOVE level");
				label_PropertyVal->m_fontColor.setBlack();
				label_PropertyName->m_fontColor.setBlack();
			}
		}
		else if (temp == true && modality_temp == 3) {
			float Number = round(temperature);       // number to be converted to a string, rounded to nearest integer//round(temperature * 10) / 10; // rounds the update float to 1 decimal place
			convert << Number;      // insert the textual representation of 'Number' in the characters in the stream
			Result = convert.str(); // set 'Result' to the contents of the stream
			label_PropertyVal->setText(Result); // sets the result as string to the label
			label_PropertyVal->setFontScale(4); // sets font size

			if (97 <= temperature && temperature <= 99)
			{
				world2->m_backgroundColor.set(1, 1, 1);
				label_PropertyVal->m_fontColor.setBlack();
				label_PropertyName->m_fontColor.setBlack();
			}
			else if (temperature < 97 && temperature >= 60)
			{
				world2->m_backgroundColor.set(1 - (1.25 - ((temperature) / 98.5)), 1 - (1.25 - ((temperature) / 98.5)), 1);
				label_PropertyVal->m_fontColor.setBlack();
				label_PropertyName->m_fontColor.setBlack();
			}
			else if (temperature <= 140 && temperature > 99)
			{
				world2->m_backgroundColor.set(1, 1 - (((temperature) / 98.5) - 0.75), 1 - (((temperature) / 98.5) - 0.75));
				label_PropertyVal->m_fontColor.setBlack();
				label_PropertyName->m_fontColor.setBlack();
			}

			else if (temperature > 140)
			{
				world2->m_backgroundColor.set(0, 0, 0);
				label_PropertyVal->setText("Temperature ABOVE device limit");
				label_PropertyVal->m_fontColor.setWhite();
				label_PropertyName->m_fontColor.setWhite();
			}

			else if (temperature < 60)
			{
				world2->m_backgroundColor.set(0, 0, 0);
				label_PropertyVal->setText("Temperature BELOW device limit");
				label_PropertyVal->m_fontColor.setWhite();
				label_PropertyName->m_fontColor.setWhite();
			}
		}
		else {
			world2->m_backgroundColor.set(0, 0, 0);
			label_PropertyVal->setText("Off");
			label_PropertyVal->m_fontColor.setWhite();
			label_PropertyVal->setFontScale(3) ; // sets font size
			label_PropertyName->m_fontColor.setWhite();

		}

		label_PropertyVal->setLineSpacing(20);
		window2SizeCallback(window2, width2, height2);


		// set base image height
		double base_h = 0.5 * height2;
		double base_w = ratioMax * base_h;

		// set distance between images
		double distance = 1.3 * base_w;

		/////////////////////////////////////////////////////////////////////
		// RENDER SCENE
		/////////////////////////////////////////////////////////////////////

		// update shadow maps (if any)
		world2->updateShadowMaps(false, mirroredDisplay);

		// render world
		camera2->renderView(width, height);

		// wait until all GL commands are completed
		glFinish();

		// check for any OpenGL errors
		GLenum err;
		err = glGetError();
		if (err != GL_NO_ERROR) cout << "Error: " << gluErrorString(err) << endl;

		// swap buffers
		glfwSwapBuffers(window2);

		// process events
		glfwPollEvents();
		Sleep(500);
	}

	// exit thread on close or simulation end
	simulationFinished = true;
	cout << "Simulation finished" << endl;
}

void window2SizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
	// update window size
	width2 = a_width;
	height2 = a_height;

	// update position of labels
	label_PropertyVal->setLocalPos((int)(0.5 * (width2 - label_PropertyVal->getWidth())), (0.5 * (height2 - label_PropertyVal->getHeight()))); // change the position of the label so that it is in the center
	label_PropertyName->setLocalPos((int)((0.5 * (width2 - label_PropertyName->getWidth()))- ((label_PropertyName->getWidth()))), (0.75 * (height2 - label_PropertyName->getHeight()))); // change the position of the label so it is in the center
}
//-------------------------------------------------------------------------------------------------------------

DWORD WINAPI updatePelt(LPVOID lpParam) {

	// simulation in now running

	simulationRunning = true;
	simulationFinished = false;

	while (simulationRunning == true) {

		if (temp == true && modality_temp == 4) {		
 
			if (temperature > 60) {
				const int WLEN = 4;
				char cwriteBuffer[WLEN + 1];
				memset(cwriteBuffer, '\0', WLEN + 1 );
				char cwrite_Str[WLEN + 1];

				_itoa_s(temperature, cwrite_Str, 10);
				strcat(cwriteBuffer, cwrite_Str);
				strcat(cwriteBuffer, "\n");
				Sleep(500);

				cout << "send to serial:" << cwriteBuffer << endl;
				if (!peltWriteSerial.WriteData(cwriteBuffer, WLEN)) {
					cout << "Didn't send to peltier!" << endl;
					//return false;
				}

			}			
		}

	}
	// exit thread
	simulationFinished = true;
	return(0);
}