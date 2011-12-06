#pragma once

#include "ofxOpenCv.h"
#include "ofxCvConvexityDefects.h"
#include "myGUI\MyGUI.h"

class HandTracking{
public:
	HandTracking();
	void detect(unsigned char* depthPixels);
	void drawSetup();
	void draw();
	vector<ofxCvBlobWithDefects>& getBlobs();
	void filterDefects();
	
	ofxCvGrayscaleImage& getThreshImg(){
		return threshImg;
	}
	
protected:
	ofxCvGrayscaleImage depthImage;
	float threshold;
	float minSize, maxSize;
	float minDepth, maxDepth;
	MyGUI* gui;
	ofxCvConvexityDefects convexityDefects;
	ofxCvGrayscaleImage threshImg, prevImg;
	ofPoint avgFlow, smoothFlow;
};