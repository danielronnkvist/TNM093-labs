#include "sgct.h"
#include <sstream>

sgct::Engine * gEngine;

//-----------------------
// function declarations 
//-----------------------
void myInitOGLFun();
void myPreSyncFun();
void myDrawFun();

//for syncing variables across a cluster
void myEncodeFun();
void myDecodeFun();

void drawAxes(float size);
void drawWireCube(float size);
int calculateIntersection(sgct::SGCTTrackingDevice *wand);
//-----------------------
// variable declarations 
//-----------------------

//store each device's transform 4x4 matrix in a shared vector
sgct::SharedVector<glm::mat4> sharedTransforms;
sgct::SharedString sharedText;
sgct::SharedObject<size_t> sharedHeadSensorIndex(0);
sgct::SharedVector<bool> sharedSelect;
sgct::SharedBool sharedbuttonPressed;

glm::vec3 trackPos;
glm::vec3 trackrot;
//pointer to a device
sgct::SGCTTrackingDevice * devicePtr = NULL;
//pointer to a tracker
sgct::SGCTTracker * trackerPtr = NULL;

int main( int argc, char* argv[] )
{
	// Allocate
	gEngine = new sgct::Engine( argc, argv );

	// Bind your functions
	gEngine->setInitOGLFunction( myInitOGLFun );
	gEngine->setPreSyncFunction( myPreSyncFun );
	gEngine->setDrawFunction( myDrawFun );

	// Init the engine
	if( !gEngine->init() )
	{
		delete gEngine;
		return EXIT_FAILURE;
	}

	sgct::SharedData::instance()->setEncodeFunction( myEncodeFun );
	sgct::SharedData::instance()->setDecodeFunction( myDecodeFun );

	// Main loop
	gEngine->render();

	// Clean up
	delete gEngine;

	// Exit program
	exit( EXIT_SUCCESS );
}

void myInitOGLFun()
{
	glEnable(GL_DEPTH_TEST);

	for (int i = 0; i < 37; i++)
	{
		sharedSelect.addVal(false);
	}

	//only store the tracking data on the master node
	if( gEngine->isMaster() )
	{
		size_t index = 0;
		
		//allocate shared data
		for(size_t i = 0; i < sgct::Engine::getTrackingManager()->getNumberOfTrackers(); i++)
		{
			trackerPtr = sgct::Engine::getTrackingManager()->getTrackerPtr(i);
			
			//init the shared vector with identity matrixes
			for(size_t j=0; j<trackerPtr->getNumberOfDevices(); j++)
			{
				devicePtr = trackerPtr->getDevicePtr(j);
			
				if( devicePtr->hasSensor() )
				{
					sharedTransforms.addVal( glm::mat4(1.0f) );
					
					//find the head sensor
					if( sgct::Engine::getTrackingManager()->getHeadDevicePtr() == devicePtr )
						sharedHeadSensorIndex.setVal(index);

					index++;
				}
			}
		}
	}
}

/*
	This callback is called once per render loop iteration.
*/
void myPreSyncFun()
{
	/*
	Store all transforms in the array by looping through all trackers and all devices.

	Storing values from the tracker in the pre-sync callback will guarantee
	that the values are equal for all draw calls within the same frame.
	This prevents the application from getting different tracked data for
	left and right draws using a stereoscopic display. Remember to get
	all sensor, button and analog data that will affect the rendering in this stage.
	*/

	//only store the tracking data on the master node
	if( gEngine->isMaster() )
	{
		size_t index = 0;
		std::stringstream ss;
		/*
			Loop trough all trackers (like intersense IS-900, Microsoft Kinect, PhaseSpace etc.)
		*/
		for(size_t i = 0; i < sgct::Engine::getTrackingManager()->getNumberOfTrackers(); i++)
		{
			trackerPtr = sgct::Engine::getTrackingManager()->getTrackerPtr(i);
		
			
			/*
				Loop trough all tracking devices (like headtracker, wand, stylus etc.)
			*/
			for(size_t j = 0; j < trackerPtr->getNumberOfDevices(); j++)
			{
				devicePtr = trackerPtr->getDevicePtr(j);
				
				ss << "Device " << i <<  "-" << j << ": " << devicePtr->getName() << "\n";
				
				if( devicePtr->hasSensor() )
				{
					sharedTransforms.setValAt( index, devicePtr->getWorldTransform() );
					index++;

					double trackerTime = devicePtr->getTrackerDeltaTime();
					ss << "     Sensor id: " << devicePtr->getSensorId()
						<< " freq: " << (trackerTime <= 0.0 ? 0.0 : 1.0/trackerTime) << " Hz\n";

					ss << "\n     Pos\n"
						<< "          x=" << devicePtr->getPosition().x << "\n"
						<< "          y=" << devicePtr->getPosition().y << "\n"
						<< "          z=" << devicePtr->getPosition().z << "\n";

					ss << "\n     Rot\n"
						<< "          rx=" << devicePtr->getEulerAngles().x << "\n"
						<< "          ry=" << devicePtr->getEulerAngles().y << "\n"
						<< "          rz=" << devicePtr->getEulerAngles().z << "\n";
				}

				if( devicePtr->hasButtons() )
				{
					ss << "\n     Buttons\n";
					
					for(size_t k=0; k < devicePtr->getNumberOfButtons(); k++)
					{
						ss << "          Button " << k << ": " << (devicePtr->getButton(k) ? "pressed" : "released") << "\n";
					}

					bool pressed = devicePtr->getButton(5);
					sharedbuttonPressed.setVal(pressed);
					if (pressed)
					{
						int cubeIndex = calculateIntersection(devicePtr);
						sharedSelect.setValAt(cubeIndex, !sharedSelect.getValAt(cubeIndex));
						trackPos = devicePtr->getPosition();
						trackrot = devicePtr->getEulerAngles();
						//sharedSelect.setValAt(15, !sharedSelect.getValAt(15));
						//sharedSelect.setValAt(7, !sharedSelect.getValAt(7));
						//sharedSelect.setValAt(20, !sharedSelect.getValAt(20));
						//sharedSelect.setValAt(8, !sharedSelect.getValAt(8));
					}

				}

				if( devicePtr->hasAnalogs() )
				{
					ss << "\n     Analog axes\n";
					
					for(size_t k=0; k < devicePtr->getNumberOfAxes(); k++)
					{
						ss << "          Axis " << k << ": " << devicePtr->getAnalog(k) << "\n";
					}
				}

				ss << "\n";
			}
		}

		//store the string stream into the shared string
		sharedText.setVal( ss.str() );
	}
}

/*
	This callback can be called several times per render loop iteration.
	Using a single viewport in stereo (3D) usually results in refresh rate of 120 Hz.
*/
void myDrawFun()
{
	int cube = 0;
	//draw some yellow cubes in space
	for( float i=-0.5f; i<=0.5f; i+=0.2f)
		for(float j=-0.5f; j<=0.5f; j+=0.2f)
		{
			cube++;
			glPushMatrix();
			glTranslatef(i, j, 0.0f);
			glTranslatef(0, 0, 0.1f*sin(gEngine->getTime() + j + i));
			if (sharedSelect.getValAt(cube))
				glColor3f(1.0f, 1.0f, 1.0f);
			else
				glColor3f(1.0f,1.0f,0.0f);
			drawWireCube(0.04f);
			glPopMatrix();
		}

	//draw a cube and axes around each wand
	for(size_t i = 0; i < sharedTransforms.getSize(); i++)
	{
		if(i != sharedHeadSensorIndex.getVal()) 
		{
			glLineWidth(2.0);

			glPushMatrix();

			glMultMatrixf( glm::value_ptr( sharedTransforms.getValAt( i ) ) );

			glColor3f(0.5f,0.5f,0.5f);
			drawWireCube(0.1f);

			drawAxes(0.1f);

			//draw pointer line
			glBegin(GL_LINES);
			if (sharedbuttonPressed.getVal())
				glColor3f(1.0f, 0.0f, 0.0f);
			else
				glColor3f(1.0f,1.0f,0.0f);
			glVertex3f(0.0f, 0.0f, 0.0f);
			glVertex3f(0.0f, 0.0f, -5.0f);
			glEnd();

			glPopMatrix();
		}
	}

	//draw text
	float textVerticalPos = static_cast<float>(gEngine->getActiveWindowPtr()->getYResolution()) - 100.0f;
	int fontSize = 12;
	
	glColor3f(1.0f, 1.0f, 1.0f);
	sgct_text::print(sgct_text::FontManager::instance()->getFont( "SGCTFont", fontSize ),
		120.0f, textVerticalPos,
		sharedText.getVal().c_str() );
}

void myEncodeFun()
{
	sgct::SharedData::instance()->writeVector( &sharedTransforms );
	sgct::SharedData::instance()->writeVector(&sharedSelect);
	sgct::SharedData::instance()->writeString( &sharedText );
	sgct::SharedData::instance()->writeObj(&sharedHeadSensorIndex);
	sgct::SharedData::instance()->writeBool(&sharedbuttonPressed);
}

void myDecodeFun()
{
	sgct::SharedData::instance()->readVector( &sharedTransforms );
	sgct::SharedData::instance()->readVector(&sharedSelect);
	sgct::SharedData::instance()->readString( &sharedText );
	sgct::SharedData::instance()->readObj(&sharedHeadSensorIndex);
	sgct::SharedData::instance()->readBool(&sharedbuttonPressed);
}

void drawAxes(float size)
{
	glLineWidth(2.0);
	glBegin(GL_LINES);

	//x-axis
	glColor3f(1.0f,0.0f,0.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(size, 0.0f, 0.0f);

	//y-axis
	glColor3f(0.0f,1.0f,0.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, size, 0.0f);

	//z-axis
	glColor3f(0.0f,0.0f,1.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, size);

	glEnd();
}

void drawWireCube(float size)
{
	//bottom
	glBegin(GL_LINE_STRIP);
	glVertex3f( -size, -size, -size);
	glVertex3f( size, -size, -size);
	glVertex3f( size, -size, size);
	glVertex3f( -size, -size, size);
	glVertex3f( -size, -size, -size);
	glEnd();

	//top
	glBegin(GL_LINE_STRIP);
	glVertex3f( -size, size, -size);
	glVertex3f( size, size, -size);
	glVertex3f( size, size, size);
	glVertex3f( -size, size, size);
	glVertex3f( -size, size, -size);
	glEnd();

	//sides
	glBegin(GL_LINES);
	glVertex3f( -size, -size, -size);
	glVertex3f( -size, size, -size);

	glVertex3f( size, -size, -size);
	glVertex3f( size, size, -size);

	glVertex3f( size, -size, size);
	glVertex3f( size, size, size);

	glVertex3f( -size, -size, size);
	glVertex3f( -size, size, size);
	glEnd();
}

int calculateIntersection(sgct::SGCTTrackingDevice *wand)
{
	glm::vec4 wandLine(0, 0, -50, 0);
	glm::mat4 wandTransform = wand->getWorldTransform();
	wandLine = wandTransform * wandLine;
	glm::vec4 wandPos(wand->getPosition(), 0);
	int cube = 0;
	float cubeRadius = 0.04f;

	for (float i = -0.5f; i <= 0.5f; i += 0.2f)
	{
		for (float j = -0.5f; j <= 0.5f; j += 0.2f)
		{
			glm::vec3 cubePos(i, j, 0);

			


			if (false)
				return cube;
			cube++;
		}
	}
	return cube;
}