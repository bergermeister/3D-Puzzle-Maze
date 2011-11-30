#include "HandTracking.h"


HandTracking::HandTracking(){
	threshold = 50;
	depthImage.allocate( 640, 480 );
	threshImg.allocate( 640, 480 );
	prevImg.allocate(640, 480);
	
	minSize = 0.0f;
	maxSize = 100000.0f;
	
	minDepth = 10.0f;
	maxDepth = 1000.0f;
	
	threshold = 80;
	
	gui = new MyGUI();
	MyPanel* settingsPanel = new MyPanel("Settings", 700, 20);
	gui->addControl( settingsPanel );
	
	MySlider* thresholdSlider = new MySlider("Threshold", 10, 30, 200, 20, &threshold, 0, 255);
	settingsPanel->addControl( thresholdSlider );
	
	MySlider* minSizeSlider = new MySlider("Min Size", 10, 60, 200, 20, &minSize, 0.0f, 10000.0f);
	settingsPanel->addControl( minSizeSlider );
	MySlider* maxSizeSlider = new MySlider("Max Size", 10, 90, 200, 20, &maxSize, 0.0f, 640*480);
	settingsPanel->addControl( maxSizeSlider );
	
	MySlider* minDepthSlider = new MySlider("Min Depth", 10, 120, 200, 20, &minDepth, 0.0f, 100.0f);
	settingsPanel->addControl( minDepthSlider );
	MySlider* maxDepthSlider = new MySlider("Max Slider", 10, 150, 200, 20, &maxDepth, 0.0f, 100.0f);
	settingsPanel->addControl( maxDepthSlider );
}

void HandTracking::detect(unsigned char* depthPixels){
	depthImage.setFromPixels( depthPixels, 640, 480);
	threshImg = depthImage;
	threshImg.threshold( threshold );
	convexityDefects.findContoursAndDefects(threshImg, minSize, maxSize, 10, false, true);
	prevImg = threshImg;	
}

void HandTracking::drawSetup(){
	glColor3f(1.0f, 1.0f, 1.0f);
	depthImage.draw(20, 20);
	
	convexityDefects.draw( 20, 20 );
	
	glColor3f(1.0f, 1.0f, 1.0f);
	threshImg.draw(680, 200, 320, 240);
	ofNoFill();
	gui->draw();
}

void HandTracking::draw(){
	
	vector<ofxCvBlobWithDefects>& blobs = convexityDefects.getBlobs();
	if(blobs.size() == 0) return;
	vector<ofPoint>& contour = blobs.front().pts;
	
	float scaleX = ofGetWidth() / threshImg.getWidth();
	float scaleY = ofGetHeight() / threshImg.getHeight();
	glPushMatrix();
	glScalef( scaleX, scaleY, 1.0f );
	glColor3f(0.3f, 0.3f, 0.3f);
	ofBeginShape();
	for( vector<ofPoint>::iterator it = contour.begin(); it != contour.end(); ++it ){
		ofPoint& pt = *it;
		ofVertex( pt.x, pt.y );
	}
	ofEndShape();
	glPopMatrix();
}

vector<ofxCvBlobWithDefects>& HandTracking::getBlobs(){
	return convexityDefects.getBlobs();
}

void HandTracking::filterDefects(){
	vector<ofxCvBlobWithDefects>& blobs = convexityDefects.getBlobs();
	for(vector<ofxCvBlobWithDefects>::iterator it = blobs.begin(); it != blobs.end(); ++it){
		ofxCvBlobWithDefects& blob = *it;
		vector<ofxCvDefect>& defects = blob.defects;
		for(vector<ofxCvDefect>::iterator defIt = defects.begin(); defIt != defects.end();){
			ofxCvDefect& defect = *defIt;
			ofPoint center = (defect.startPoint + defect.endPoint) * 0.5f;
			center -= defect.depthPoint;
			float dist = sqrtf( center.x *center.x + center.y*center.y );
			if(dist < maxDepth && dist > minDepth){
				++defIt;
			} else{
				defects.erase( defIt );
			}
		}
	}
}
