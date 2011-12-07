#include "testApp.h"
#include <gl\glu.h>

static float smoothPct = 0.75f;
static int tolerance = 4;
bool flag=false;
int footF=0;
int directionF=0;
/*
1 = left
2 = right
1 = foward
2 = back
*/


void ode::reset()
{	
		world = dWorldCreate();
		space = dSweepAndPruneSpaceCreate(0, dSAP_AXES_XYZ );
		
		dWorldSetGravity (world,0,300,0);
		dWorldSetCFM (world, 1e-5);
		dWorldSetERP (world, 0.8);
		//dWorldSetQuickStepNumIterations (world,ITERS);
		ground = dCreatePlane (space,0,0,100,0);

}
void ode::newBall(int xp, int yp)
{ 
		dReal z=1,y=1,x=1;
	
		b = dBodyCreate (world);
		dBodySetPosition (b,xp,yp,2);
		dMassSetSphere (&m,1,RADIUS);
		dMassAdjust (&m, 1);
		dBodySetMass (b,&m);
		sphere[spheres] = dCreateSphere (space,RADIUS);
		dGeomSetBody (sphere[spheres++],b);		
}
void ode::newLine(float xp, float yp)
{
		b = dBodyCreate (world);
		dBodySetPosition (b,xp,yp,2);
		//dMassSetSphere (&m,1,RADIUS);
		dMassAdjust (&m, 1);
		dBodySetMass (b,&m);
		lineStrips[lines] = dCreateSphere (space,RADIUS);
		dGeomSetBody (lineStrips[lines++],b);		
}
void ode::drawBall()
{
	dsSetTexture (DS_WOOD);
	dWorldQuickStep (world,0.05);

	dsSetColor(0.4,0,0);

	for (int i=0; i< spheres; i++)
	{ 
		dsDrawSphere (dGeomGetPosition(sphere[i]),dGeomGetRotation(sphere[i]),RADIUS);
	}
	
}
void ode::drawLines()
{
	dsSetTexture (DS_WOOD);
	//dWorldQuickStep (world,0.05);

	dsSetColor(1,0,0);
	//const dReal *temp1=dGeomGetPosition(lineStrips[0]);
	
	for (int i=0; i< contourX.size(); i++)
	{ 
		float tempX[3];
		tempX[0]=contourX[i][0];
		tempX[0]=contourX[i][1];
		tempX[0]=contourX[i][2];
		float tempY[3];
		tempY[0]=contourY[i][0];
		tempY[0]=contourY[i][1];
		tempY[0]=contourY[i][2];
		//dsDrawLine(tempX,tempY);
	}
	flag=false;
}


//--------------------------------------------------------------
void testApp::setCalibrationOffset(float x, float y) {
	ofxMatrix4x4 matrix = kinect.getRGBDepthMatrix();
	matrix.getPtr()[3] = x;
	matrix.getPtr()[7] = y;
	kinect.setRGBDepthMatrix(matrix);
}

void testApp::drawFigures()
{
	switch(directionF){
		case 1:  //left
			glColor3f(1.0, 0.0, 0.0);    //red
			glBegin(GL_QUADS);
			glTexCoord2f(0, 0);
			glVertex3f(780, 780, 0.0f);
			glTexCoord2f(1, 0);
			glVertex3f(790, 780, 0.0f);
			glTexCoord2f(1, 1);
			glVertex3f(790, 590, 0.0f);
			glTexCoord2f(0, 1);
			glVertex3f(780, 590, 0.0f);
			glEnd();
			if(shm->h >= 360)
				shm->h = 0;
			else
				shm->h+=5;
			break;
		case 2:  //right
			glColor3f(0.0, 0.0, 1.0);    //blue
			glBegin(GL_QUADS);
			glTexCoord2f(0, 0);
			glVertex3f(780, 780, 0.0f);
			glTexCoord2f(1, 0);
			glVertex3f(790, 780, 0.0f);
			glTexCoord2f(1, 1);
			glVertex3f(790, 590, 0.0f);
			glTexCoord2f(0, 1);
			glVertex3f(780, 590, 0.0f);
			glEnd();
			if(shm->h <= 0)
				shm->h = 360;
			else
				shm->h-= 5;
			break;
	}
	
	switch(footF){
	case 1:   //up
		glColor3f(0.0, 1.0, 1.0);    //light blue
		glBegin(GL_QUADS);
		glTexCoord2f(0, 0);
		glVertex3f(800, 800, 0.0f);
		glTexCoord2f(1, 0);
		glVertex3f(790, 800, 0.0f);
		glTexCoord2f(1, 1);
		glVertex3f(790, 590, 0.0f);
		glTexCoord2f(0, 1);
		glVertex3f(800, 590, 0.0f);
		glEnd();
		shm->x += cos(shm->h * PI / 180);
		shm->y += sin(shm->h * PI / 180);
		//shm->s+=0.3;
		break;
	case 2:   //back -- stop
		glColor3f(0.0, 0.0, 0.0);    //black
		glBegin(GL_QUADS);
		glTexCoord2f(0, 0);
		glVertex3f(800, 800, 0.0f);
		glTexCoord2f(1, 0);
		glVertex3f(790, 800, 0.0f);
		glTexCoord2f(1, 1);
		glVertex3f(790, 590, 0.0f);
		glTexCoord2f(0, 1);
		glVertex3f(800, 590, 0.0f);
		shm->s = 0;
		glEnd();  
		break;
	
	}
}
//--------------------------------------------------------------
void testApp::setup() {

	physics.spheres=0;
	ofSetLogLevel(OF_LOG_VERBOSE);
	
    // enable depth->rgb image calibration
	kinect.setRegistration(true);
	kinect.init();
	kinect.setVerbose(true);
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)
	
	kinect.open();

	//Allocate space for all images
	colorImg.allocate(kinect.width, kinect.height);
	colorDiff.allocate(kinect.width, kinect.height);
	colorBg.allocate(kinect.width, kinect.height);

	grayImage.allocate(kinect.width, kinect.height);
	grayBg.allocate(kinect.width, kinect.height);
	grayDiff.allocate(kinect.width, kinect.height);
	footDiff.allocate(kinect.width, kinect.height );

	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
	maskedImg.allocate(kinect.width, kinect.height,GL_RGBA);
	
	// Don't capture the background at startup
	bLearnBakground = false;

	// set up sensable defaults for threshold and calibration offsets
	// Note: these are empirically set based on my kinect, they will likely need adjusting
	threshold = 73;
	
	xOff = 13.486656;
	yOff = 34.486656;	
	setCalibrationOffset(xOff, yOff);

	nearThreshold = 250;
	farThreshold = 70;
	bThreshWithOpenCV = true;
	
	// Set depth map so near values are higher (white)
	kinect.enableDepthNearValueWhite(true);
	// Set which direction virtual camera is animating
	eyeDir = 1;

	ofSetFrameRate(30);
	
	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);
	
	// start from the front
	bDrawPointCloud = false;
	bDrawDepthMap = false;
	bDrawGrayImage = false;
	bDrawContourFinder = false;
	bDrawGrayDiff = false;
	bDrawColorDiff = false;

	physics.reset();
	
}

//--------------------------------------------------------------
void testApp::update() {
	
	ofBackground(100, 100, 100);
	kinect.update();
	
	//clearing old collision lines
	physics.lines=0;

	// load grayscale depth image from the kinect source
	grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
	colorImg.setFromPixels(kinect.getPixels(), kinect.width, kinect.height);
	colorImg.getCvImage();

	// Quick and dirty noise filter on the depth map. Needs work
	grayImage.dilate();
	grayImage.erode();
	
	// If the user pressed spacebar, capture the depth and RGB images and save for later
	if (bLearnBakground == true)
	{
		grayBg = grayImage;
		colorBg = colorImg;
		bLearnBakground = false;
	}
	// Subtract the saved background from the current one
	grayDiff = grayImage;
	grayDiff -= grayBg;
	colorDiff = colorImg;
	colorDiff-=colorBg;
			
	grayDiff.threshold(1);    // anything that is > 1 has changed, so keep it
	grayDiff *= grayImage;    // multiply in the current depth values, to mask it 
	//colorDiff *=colorImg;
	//grayDiff.threshold(threshold);  // cut off anything that is too far away

	// Copy the filtered depthmap so we can use it for detecting feet 
	footDiff= grayDiff;

	// for feet we want to focus on only the bottom part of the image (et the region of interest to the bottom 180 px)
	footDiff.setROI(0, 300,footDiff.width, footDiff.height/2);
	
	// cut off anything that is too far away
    grayDiff.threshold(farThreshold); // TODO: This should be configurable as well
	footDiff.threshold(farThreshold);

	// since we set ROI, we need to reset it
	footDiff.resetROI();
	// also, since ROI was on when we did the above threshold we clear out all pixels that are not fully white 
	//(which ends up being only the upper part of the iamge)
	footDiff.threshold(nearThreshold);
	
	
	// Find blobs (should be hands and foot) in the filtered depthmap
	contourFinder.findContours(grayDiff, 1000, (kinect.width*kinect.height)/2, 5, false);
	footContourFinder.findContours(footDiff, 1000, (kinect.width*kinect.height)/2, 5, false);
	
	// if at least 2 blobs were detected (presumably 2 hands), figure out
	// their locations and calculate which way to "move"
	if (contourFinder.blobs.size() >= 2) 
	{
		// Find the x,y cord of the center of the first 2 blobs
		float x1 = contourFinder.blobs[0].centroid.x;
		float y1 = contourFinder.blobs[0].centroid.y;
		float x2 = contourFinder.blobs[1].centroid.x;
		float y2 = contourFinder.blobs[1].centroid.y;
		
		// the x1<x2 check is to ensure that p1 is always the leftmost blob (right hand)
		ofPoint p1(x1<x2 ? x1 : x2,x1<x2 ? y1 : y2, 0);
		ofPoint p2(x2<x1 ? x1 : x2,x2<x1 ? y1 : y2, 0);

		// if the "steering wheel" is sufficently rotated
		if(abs(p1.y-p2.y) > 50)
		{
			if(p1.y < p2.y ){ // turning left
				if(!leftDown){ // if left is already down, dont send key even again
					// Send the key down event for left, and up event for right
					directionF=1;
					leftDown = true;
					rightDown = false;
				}
			}
			else  // turning right
			{ 
				if(!rightDown){ // if left is already down, dont send key even again
					// Send the key down event for right, and up event for left
					directionF=2;
					rightDown = true;
					leftDown = false;
				}
			}
		} 
		else  // "steering weheel" centered so moving straight
		{
			if(leftDown){
				directionF=0;
				leftDown = false;
			}
			if(rightDown){
				directionF=0;
				rightDown = false;
			}
		}
	}
	// no hands detected
	else 
	{ 
		if(leftDown){
			footF=0;
			directionF=0;
			leftDown = false;
		}
		if(rightDown){
			footF=0;
			directionF=0;
			rightDown = false;
		}
	}
	// if any blob is detected in the foot map, it can be considered a foot
	if(footContourFinder.blobs.size() >= 1) 
	{
		//ofBackground(0,255,0); // set background to green for debugging
		if(!footDown) {   //moving foward
			footF= 1;
			footDown = true;
		}
	} 
	else 
	{
		ofBackground(100,100,100);
		if(footDown) {
			footF=2;   //stop
			footDown = false;
		}
	}
	// update the cv images
	grayImage.flagImageChanged();
	

}

//--------------------------------------------------------------
void testApp::draw() {
	
	ofSetColor(255, 255, 255);

	// Draw some debug images along the top
	kinect.drawDepth(10, 10, 315, 236);
	grayDiff.draw(335, 10, 315, 236);
	footDiff.draw(660, 10, 315, 236);
	
	// Draw a larger image of the calibrated RGB camera and overlay the found blobs on top of it
	colorImg.draw(10,256);
	contourFinder.draw(10,256);
	//footContourFinder.draw(10,256);
		
	// Display some debugging info
	char reportStr[1024];
	//sprintf(reportStr, "left: %i right: %i foot: %i", leftDown, rightDown, footDown);
	sprintf(reportStr, "left: %f right: %f foot: %f", shm->x, shm->y, shm->z);
	ofDrawBitmapString(reportStr, 20, 800);
	
	drawFigures();
	
	if(bDrawPointCloud) 
	{
		easyCam.begin();
		drawPointCloud();
		easyCam.end();
	} 
	
	else if(bDrawContourFinder) { contourFinder.draw(0,0, 1024, 768);}
	
	if(physics.spheres>0){physics.drawBall();}
	if(physics.lines>0 && flag==true){physics.drawLines();}

	// draw instructions
	ofSetColor(255, 255, 255);
	stringstream reportStream;
	reportStream << "accel is: " << ofToString(kinect.getMksAccel().x, 2) << " / "
	<< ofToString(kinect.getMksAccel().y, 2) << " / "
	<< ofToString(kinect.getMksAccel().z, 2) << endl
	<< "press p to switch between images and point cloud, rotate the point cloud with the mouse" << endl
	<< "using opencv threshold = " << bThreshWithOpenCV <<" (press spacebar)" << endl
	<< "set near threshold " << nearThreshold << " (press: + -)" << endl
	<< "set far threshold " << farThreshold << " (press: < >) num blobs found " << contourFinder.nBlobs
	<< ", fps: " << ofGetFrameRate() << endl
	<< "press c to close the connection and o to open it again, connection is: " << kinect.isConnected() << endl
	<< "press UP and DOWN to change the tilt angle: " << angle << " degrees" << endl;
	ofDrawBitmapString(reportStream.str(),20,652);
	

}

//--------------------------------------------------------------
void testApp::drawPointCloud() {
	int w = 640;
	int h = 480;
	ofMesh mesh;
	mesh.setMode(OF_PRIMITIVE_POINTS);
	int step = 2;
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
			if(kinect.getDistanceAt(x, y) > 0) {
				mesh.addColor(kinect.getColorAt(x,y));
				mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
			}
		}
	}
	glPointSize(3);
	ofPushMatrix();
	// the projected points are 'upside down' and 'backwards' 
	ofScale(1, -1, -1);
	ofTranslate(0, 0, -1000); // center the points a bit
	glEnable(GL_DEPTH_TEST);
	mesh.drawVertices();
	glDisable(GL_DEPTH_TEST);
	ofPopMatrix();
}

//--------------------------------------------------------------
void testApp::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();

}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
	switch (key) {
		case ' ':
			bThreshWithOpenCV = !bThreshWithOpenCV;
			bDrawColorDiff=false;
			break;	
		case 'b':
			bLearnBakground = true;
			break;
		case'p':
			bDrawPointCloud = !bDrawPointCloud;
			bDrawContourFinder = false;
			bDrawColorDiff=false;
			break;
		case 'c':
			bDrawContourFinder=!bDrawContourFinder;
			bDrawPointCloud = false;
			bDrawColorDiff=false;
			break;
		case'i':
			bDrawColorDiff = !bDrawColorDiff;
			bDrawPointCloud = false;
			bDrawContourFinder = false;
		break;
		case 'x':
			flag=true;
			break;
		case '>':
		case '.':
			farThreshold ++;
			if (farThreshold > 255) farThreshold = 255;
			break;
			
		case '<':
		case ',':
			farThreshold --;
			if (farThreshold < 0) farThreshold = 0;
			break;
			
		case '+':
		case '=':
			nearThreshold ++;
			if (nearThreshold > 255) nearThreshold = 255;
			break;
			
		case '-':
			nearThreshold --;
			if (nearThreshold < 0) nearThreshold = 0;
			break;
			
		case 'w':
			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;
			
		case 'a':
			kinect.setCameraTiltAngle(angle); // go back to prev tilt
			kinect.open();
			break;			
		case 'z':
			kinect.setCameraTiltAngle(0); // zero the tilt
			kinect.close();
			break;
		case 'q':
			physics.newBall(mouseX, mouseY);	
			break;
		case 'h':
			bSetup = !bSetup;
			break;
		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			break;	
		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
			break;
			case '[':
			yOff++;
			setCalibrationOffset(xOff, yOff);
			break;
		case ']':
			yOff--;
			setCalibrationOffset(xOff, yOff);
			break;
		case OF_KEY_LEFT:
			xOff--;
			setCalibrationOffset(xOff, yOff);
			break;
		case OF_KEY_RIGHT:
			xOff++;
			setCalibrationOffset(xOff, yOff);
			break;
	}
}

//--------------------------------------------------------------
void testApp::keyReleased(int key){

}
//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{
	eyeX = x-(ofGetWidth()/2);
	eyeY = y-(ofGetHeight()/2);
}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{}


