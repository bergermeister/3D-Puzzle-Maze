#include "testApp.h"
#include <gl\glu.h>

static float smoothPct = 0.75f;
static int tolerance = 4;
bool flag=false;
int footF=0;
int directionF=0;
/*
1 = right
2 = left
1 = foward
2 = back
*/


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
		case 1:  //right
			glColor3f(1.0, 0.0, 0.0);    //red
			glBegin(GL_QUADS);
				glTexCoord2f(0, 0);
				glVertex3f(200, 850, 0.0f);
				glTexCoord2f(1, 0);
				glVertex3f(220, 850, 0.0f);
				glTexCoord2f(1, 1);
				glVertex3f(220, 750, 0.0f);
				glTexCoord2f(0, 1);
				glVertex3f(200, 750, 0.0f);
			glEnd();
			if(shm->h <= 0)
			{
				shm->h = 360;
			}
			else
			{
				shm->h-=5;
			}
			//shm->left -= .3;
			break;
		case 2:  //left
			glColor3f(0.0, 0.0, 1.0);    //blue
			glBegin(GL_QUADS);
			glTexCoord2f(0, 0);
				glVertex3f(200, 850, 0.0f);
				glTexCoord2f(1, 0);
				glVertex3f(220, 850, 0.0f);
				glTexCoord2f(1, 1);
				glVertex3f(220, 750, 0.0f);
				glTexCoord2f(0, 1);
				glVertex3f(200, 750, 0.0f);
			glEnd();
			if(shm->h >= 360)
			{
				shm->h = 0;
			}
			else
			{
				shm->h+= 5;
			}
			//shm->left += .3;
			break;
	}
	
	switch(footF){
	case 1:   //up
		glColor3f(0.0, 1.0, 1.0);    //light blue
		glBegin(GL_QUADS);
			glTexCoord2f(0, 0);
			glVertex3f(220, 850, 0.0f);
			glTexCoord2f(1, 0);
			glVertex3f(240, 850, 0.0f);
			glTexCoord2f(1, 1);
			glVertex3f(240, 750, 0.0f);
			glTexCoord2f(0, 1);
			glVertex3f(220, 750, 0.0f);
		glEnd();  
		shm->x += cos(shm->h * PI / 180);
		shm->y += sin(shm->h * PI / 180);
		shm->s+=0.3;
		break;
	case 2:   //back -- stop
		glColor3f(0.0, 0.0, 0.0);    //black
		glBegin(GL_QUADS);
			glTexCoord2f(0, 0);
			glVertex3f(220, 850, 0.0f);
			glTexCoord2f(1, 0);
			glVertex3f(240, 850, 0.0f);
			glTexCoord2f(1, 1);
			glVertex3f(240, 750, 0.0f);
			glTexCoord2f(0, 1);
			glVertex3f(220, 750, 0.0f);
		glEnd();  
		shm->s = 0;
		
		break;
	
	}
}
//--------------------------------------------------------------
void testApp::setup() {

	ofSetLogLevel(OF_LOG_VERBOSE);
	
    // enable depth->rgb image calibration
	kinect.setRegistration(true);
	kinect.init();
	kinect.setVerbose(true);
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)
	
	kinect.open();

	//kinect.getCalibratedColorAt(60, 230);
	
	//Allocate space for all images
	colorImg.allocate(kinect.width, kinect.height);

	grayImage.allocate(kinect.width, kinect.height);
	grayBg.allocate(kinect.width, kinect.height);
	grayDiff.allocate(kinect.width, kinect.height);
	footDiff.allocate(kinect.width, kinect.height );
	handDiff.allocate(kinect.width, kinect.height );

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
	ofSetFrameRate(30);
	
	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);
	
	// start from the front
	bDrawPointCloud = false;
	bDrawContourFinder = false;

}

//--------------------------------------------------------------
void testApp::update() {
	
	ofBackground(100, 100, 100);
	kinect.update();
	
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
		bLearnBakground = false;
	}
	// Subtract the saved background from the current one
	grayDiff = grayImage;
	grayDiff -= grayBg;

	grayDiff.threshold(1);    // anything that is > 1 has changed, so keep it
	grayDiff *= grayImage;    // multiply in the current depth values, to mask it 
	// Copy the filtered depthmap so we can use it for detecting feet 
	footDiff= grayDiff;
	handDiff=grayDiff;
	
	// for feet we want to focus on only the bottom part of the image (et the region of interest to the bottom 180 px)
	footDiff.setROI(0,300,footDiff.width, footDiff.height/2);
	handDiff.setROI(0,0,handDiff.width, handDiff.height/2);
	
	// cut off anything that is too far away
     grayDiff.threshold(farThreshold);
	 footDiff.threshold(farThreshold);
	 handDiff.threshold(farThreshold);
	// since we set ROI, we need to reset it
	footDiff.resetROI();
	handDiff.resetROI();
	// also, since ROI was on when we did the above threshold we clear out all pixels that are not fully white 
	//(which ends up being only the upper part of the iamge)
	footDiff.threshold(nearThreshold);
	handDiff.threshold(nearThreshold);

	handDiff.mirror(false,true);
	footDiff.mirror(false,true);
	// Find blobs (should be hands and foot) in the filtered depthmap
	contourFinder.findContours(handDiff, 1000, (kinect.width*kinect.height)/2, 5, false);
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

			if(p1.y > p2.y ) // turning right if left hand raised
			{
				if(!leftDown){ // if left is already down, dont send key even again
					// Send the key down event for left, and up event for right

					shm->right += .5;
					shm->rightH =p1.y/300;

					shm->left -= .5;
					shm->leftH =p1.y/300;

					directionF=1;
					leftDown = true;
					rightDown = false;
				}
			}
			else if(p1.y < p2.y )  // turning left if right hand raised
			{ 
				if(!rightDown) // if left is already down, dont send key even again
				{
					// Send the key down event for right, and up event for left
					shm->left += .5;
					shm->leftH =p1.y/300;

					shm->right -= .5;
					shm->rightH =p1.y/300;

					directionF=2;
					rightDown = true;
					leftDown = false;
				}
			}
			else  // hands centered so moving straight
			{
				if(leftDown)
				{
					directionF=0;
					leftDown = false;
				}
				if(rightDown)
				{
					directionF=0;
					rightDown = false;
				}
			}
	}
	// no hands detected so moving straight
	else 
	{ 
		if(leftDown)
		{
			directionF=0;
			leftDown = false;
		}
		if(rightDown)
		{
			directionF=0;
			rightDown = false;
		}
	}
	// if any blob is detected in the foot map, it can be considered a foot
	if(footContourFinder.blobs.size() >= 1) 
	{
		//ofBackground(0,255,0); // set background to green for debugging
		if(!footDown)    //moving foward
		{
			footF= 1;
			footDown = true;
		}
	} 
	else 
	{
		//ofBackground(100,100,100);
		if(footDown) 
		{
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
	handDiff.draw(20, 256, 315, 236);
	footDiff.draw(20, 502, 315, 236);
	
	// Draw a larger image of the calibrated RGB camera and overlay the found blobs on top of it
	colorImg.mirror(false,true);
	colorImg.draw(20, 10, 315, 236);

	contourFinder.draw(20, 10, 315, 236);
	footContourFinder.draw(20, 10, 315, 236);
		
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
			break;	
		case 'b':
			bLearnBakground = true;
			break;
		case'p':
			bDrawPointCloud = !bDrawPointCloud;
			bDrawContourFinder = false;
			break;
		case 'c':
			bDrawContourFinder=!bDrawContourFinder;
			bDrawPointCloud = false;
			break;
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
{}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{}


