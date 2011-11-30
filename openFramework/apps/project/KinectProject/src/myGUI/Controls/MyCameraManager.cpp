
#ifdef USE_OFXOPENCV
#include "MyCameraManager.h"
#include "MyToggle.h"
#include "MySlider.h"
#include "MySlider2d.h"

MyCameraManager::MyCameraManager(string _name, float _x, float _y, float _previewW, float _previewH):
MyPanel(_name, _x, _y)
{
	previewW = _previewW;
	previewH = _previewH;
	bInited = false;
	bUseCam = false;
	bUseAdditiveBlend = false;
	cam = new ofVideoGrabber();
	img = new FadableCvGrayImage();
	img->setAlpha(0.0);
	img->stopFade();
	colorImg = new ofxCvColorImage();
	zoomedImg = new ofxCvGrayscaleImage();
	useCamToggle = new MyToggle("Use Cam", 10, previewH + 10, previewW, 30);
	addControl(useCamToggle);
	ofAddListener(useCamToggle->turnOn, this, &MyCameraManager::onCamToggle);
	ofAddListener(useCamToggle->turnOff, this, &MyCameraManager::onCamToggle);
	useCamToggle->turnOffToggle();
	addBlendToggle = new MyToggle("Additive Blend", 10, previewH + 50, previewW, 30);
	addControl(addBlendToggle);
	addBlendToggle->turnOffToggle();
	ofAddListener(addBlendToggle->turnOn, this, &MyCameraManager::onAddBlendToggle);
	ofAddListener(addBlendToggle->turnOff, this, &MyCameraManager::onAddBlendToggle);
	
	ofPoint minPan;
	ofPoint maxPan(1.0f, 1.0f);
	panPos.set(0.5f, 0.5f);
	panSlider2d = new MySlider2d("Pan", 10, previewH + 130, previewW, previewH, &panPos, minPan, maxPan);
	addControl(panSlider2d);
	
	zoomAmount = 0;
	float maxZoom = 5;
	float minZoom = 1;
	zoomSlider = new MySlider("Zoom", 10, previewH + 100, previewW, 20, &zoomAmount, minZoom, maxZoom);
	addControl(zoomSlider);
	glEnable(GL_BLEND);
}

MyCameraManager::~MyCameraManager(){
	clear();
}

void MyCameraManager::init(float _camW, float _camH){
	camW = _camW;
	camH = _camH;
	cam->initGrabber(camW, camH, true);
	img->allocate(camW, camH);
	colorImg->allocate(camW, camH);
	colorImg->setUseTexture(false);
	zoomedImg->allocate(camW, camH);
	zoomedImg->setUseTexture(false);
	bInited = true;
}

void MyCameraManager::clear(){
	if(!bInited)return;
	MyPanel::clear();
	if(cam){
		cam->close();
		delete cam;
	}
	if(img){
		img->clear();
		delete img;
	}
	bInited = false;
}

void MyCameraManager::update(){
	if(!bInited){
		ofLog(OF_LOG_WARNING, "MyCameraManager::update - not inited");
	}
	unsigned long currentTime = ofGetSystemTime();
	img->update(currentTime);
	cam->grabFrame();
	if(cam->isFrameNew()){
		colorImg->setFromPixels(cam->getPixels(), cam->getWidth(), cam->getHeight());
		zoomedImg->setFromColorImage(*colorImg);
		if(zoomAmount > 0.0f){
			cvResetImageROI(zoomedImg->getCvImage());
			float roiWidth = zoomedImg->getWidth() / zoomAmount;
			float roiHeight = zoomedImg->getHeight() / zoomAmount;
			float roiX = (zoomedImg->getWidth() - roiWidth) * panPos.x;
			float roiY = (zoomedImg->getHeight() - roiHeight) * panPos.y;
			cvSetImageROI(zoomedImg->getCvImage(), cvRect(roiX, roiY, roiWidth, roiHeight));
		}
		img->scaleIntoMe(*zoomedImg);
	}
}

void MyCameraManager::draw(){
	MyPanel::draw();
	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
	//img->draw(x, y, previewW, previewH);
	img->drawWithFullAlpha(x, y, previewW, previewH);
}

void MyCameraManager::drawVideo(float x, float y){
	if(!bInited)return;
	img->draw(x, y);
}

void MyCameraManager::drawVideo(float x, float y, float w, float h){
	if(!bInited)return;
	img->draw(x, y, w, h);
}

void MyCameraManager::onCamToggle(ControlEventArgs& e){
	bUseCam = ((MyToggle*)e.control)->isOn();
	if(bUseCam){
		img->fadeIn();
	} else img->fadeOut();
}

void MyCameraManager::onAddBlendToggle(ControlEventArgs& e){
	bUseAdditiveBlend = ((MyToggle*)e.control)->isOn();
}

bool MyCameraManager::usingCamera(){
	return img->getAlpha() > 0.0f;
}

bool MyCameraManager::useAdditiveBlend(){
	return bUseAdditiveBlend;
}

#endif
							

