#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofAppGlutWindow.h"
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"
#include "shared_data.h"

#define dsDrawSphere dsDrawSphereD
//#define dsDrawLine   dsDrawLineD
#define RADIUS 25		// wheel radius
#define LENGTH 3.5		// chassis length
#define WIDTH 2.5		// chassis width
#define HEIGHT 1.0		// chassis height
#define BALLMASS 1		// ball mass



class testApp : public ofBaseApp {
public:
	struct shared_data * shm;
	testApp(struct shared_data * data){shm = data;};
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

	// Used for storing each RGB frame
	ofxCvColorImage	colorImg;

	ofxCvGrayscaleImage grayImage; // grayscale depth image
	ofxCvGrayscaleImage grayBg;   // Used to store captured depth bg
	ofxCvGrayscaleImage grayDiff;    // Used to store the processed depth image
	// Used to store the processed depth image for the feet
	ofxCvGrayscaleImage footDiff;
	ofxCvGrayscaleImage handDiff;

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
	bool bDrawContourFinder;

	int nearThreshold;
	int farThreshold;
	// distance at which depth map is "cut off"
	int	threshold;

	int angle;  // Current camera tilt angle

	// The calibration offsets to align depth and RGB cameras
	float xOff;
	float yOff;
	
	// Used to store the masked RGB iamge of the forgeground object
	ofTexture maskedImg;

	// Flag to capture the background in the next update()
	bool bLearnBakground;
	
	int oldNumBlobs;

    // used for viewing the point cloud
	ofEasyCam easyCam;
};
