#pragma once

#ifdef USE_OFXOPENCV

#include "FadableCam.h"
#include "MyPanel.h"
#include "ofxOpenCv.h"
#include "FadableCvGrayImage.h"

class MyToggle;
class MySlider;
class MySlider2d;

class MyCameraManager: public MyPanel{
protected:	
	MyToggle* useCamToggle, *addBlendToggle;
	MySlider* fadeSlider, *zoomSlider;
	float zoomAmount;
	ofVideoGrabber* cam;
	ofxCvColorImage* colorImg;
	ofxCvGrayscaleImage* zoomedImg;
	FadableCvGrayImage* img;
	
	MySlider2d* panSlider2d;
	ofPoint panPos;
	float camW, camH;
	bool bInited;
	bool bUseCam;
	float previewW, previewH;
	bool bUseAdditiveBlend;
public:
	MyCameraManager(string _name, float _x, float _y, float _previewW, float _previewH);
	virtual ~MyCameraManager();
	
	virtual void init(float _camW = 320, float _camH = 240);
	virtual void clear();
	virtual void update();
	virtual void draw();
	virtual void drawVideo(float x, float y);
	virtual void drawVideo(float x, float y, float w, float h);
	
	virtual bool usingCamera();
	virtual bool useAdditiveBlend();
	
protected: //internal events
	virtual void onCamToggle(ControlEventArgs& e);
	virtual void onAddBlendToggle(ControlEventArgs& e);
};

#endif
