#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofAppGlutWindow.h"
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"
#include "contourSimplify.h"

#include "HandTracking.h"
#include "Draggable.h"

#define dsDrawSphere dsDrawSphereD
//#define dsDrawLine   dsDrawLineD
#define RADIUS 25		// wheel radius
#define LENGTH 3.5		// chassis length
#define WIDTH 2.5		// chassis width
#define HEIGHT 1.0		// chassis height
#define BALLMASS 1		// ball mass

// uncomment this to read from two kinects simultaneously
//#define USE_TWO_KINECTS
class ode {
public:
	dWorldID world;
	dSpaceID space;
	dGeomID ground;

	dBodyID body[10000];
	dBodyID b;
	dMass m;
	
	dGeomID lineStrips[10000];
	dGeomID box[10000];
	dGeomID sphere[10000];
	vector <vector<float>> contourX;
	vector <vector<float>> contourY;
	//dsSetTexture(b);
	int boxes;
	int lines;
	int spheres;
	int grav;
	void reset();
	void newBall(int x, int y);
	void drawBall();
	void drawLines();
	void newLine(float x, float y);
};


class testApp : public ofBaseApp {
public:
	void setup();
	void update();
	void draw();
	void exit();
	
	void drawPointCloud();
	// Quick utility function to apply the offsets to the camera matrix
	void setCalibrationOffset(float x, float y);
	
	void keyPressed (int key);
	void keyReleased(int key);
	
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);

	void drawFigures();
	 // Instance of the kinect object
	ofxKinect kinect;	
	ode physics;    // collision detection obj

	// Used for storing each RGB frame
	ofxCvColorImage	colorImg;
	ofxCvColorImage	colorDiff;
	ofxCvColorImage	colorBg;// Used to store the captured RGB background
	
	ofxCvGrayscaleImage grayImage; // grayscale depth image
	ofxCvGrayscaleImage grayBg;   // Used to store captured depth bg
	ofxCvGrayscaleImage grayDiff;    // Used to store the processed depth image
	// Used to store the processed depth image for the feet
	ofxCvGrayscaleImage footDiff;

	ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
	ofxCvGrayscaleImage grayThreshFar; // the far thresholded image
	
	// Used to find blobs in the filtered depthmap
	ofxCvContourFinder contourFinder;
	// Used to find blobs in the filtered foot depthmap
	ofxCvContourFinder 	footContourFinder;
	
	// state variables for the keypresses, so we dont
	// send multiple key up or key down events
	bool footDown;
	bool leftDown;
	bool rightDown;

	//bool variables to know what we drawing
	bool bThreshWithOpenCV;
	bool bDrawPointCloud;
	bool bDrawDepthMap;
	bool bDrawGrayImage;
	bool bDrawContourFinder;
	bool bDrawGrayDiff;
	bool bDrawColorDiff;

	int nearThreshold;
	int farThreshold;
	
	int angle;  // Current camera tilt angle

	// The calibration offsets to align depth and RGB cameras
	float xOff;
	float yOff;
	
	// Used to store the masked RGB iamge of the forgeground object
	ofTexture maskedImg;

	// Flag to capture the background in the next update()
	bool bLearnBakground;
	// distance at which depth map is "cut off"
	int	threshold;

	//for simple contour and become it into collision contour
	contourSimplify contourSimp;
	vector<vector <ofxPoint2f> > simpleContours;
	vector <ofxPoint2f> contourReg;
	vector <ofxPoint2f> contourSmooth;
	vector <ofxPoint2f> contourSimple;

	//collision detection variables
	HandTracking handTracking;
	ofPoint avgFlow;
	ofPoint rotation, rotationSmooth;
	
	vector<Draggable> draggables;
	
	bool bWasHandOpen;	
	bool bSetup;
	bool bHandBusy;
	int oldNumBlobs;

	// Position of the virtual camera
	GLdouble eyeX;
	GLdouble eyeY;
		
	// Which direction virtual cameara is animating
	GLdouble eyeDir;
    // used for viewing the point cloud
	ofEasyCam easyCam;
};
