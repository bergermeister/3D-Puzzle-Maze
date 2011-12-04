#ifdef USE_OFXOPENCV

#include "MyVideoPlayerPlus.h"

#include "MyButton.h"
#include "MySlider.h"
#include "MyToggle.h"
#include "MyScrollList.h"

MyVideoPlayerPlus::MyVideoPlayerPlus(string _name, float _x, float _y, float _w, float _h):
MyVideoPlayer(_name, _x, _y, _w, _h)
{
	vidA = new FadableVideo();
	vidB = new FadableVideo();
	vidA->setUseTexture(false);
	vidB->setUseTexture(false);
	vidATexture = new ofTexture();
	vidBTexture = new ofTexture();
	whichVideo = NULL;
	delete vid;
	vid = vidB;
	ofEnableAlphaBlending();
	vidB->setAlpha(0.0f);
	vidA->stopFade();
	vidB->stopFade();
	bUseFade = true;
	
	vidA->setAlpha(0.0f);
	vidA->stopFade();
	
	float maxFadeSeconds = 30.0f;
	float minFadeSeconds = 0.0f;
	fadeSeconds = 1.0f;
	
	vidA->setFadeMillis((int)fadeSeconds * 1000);
	vidB->setFadeMillis((int)fadeSeconds * 1000);
	
	fadeSecondsSlider = new MySlider("", BUTTON_PADDING_X*3 + previewW + BUTTONS_WIDTH, 80 + BUTTON_PADDING_Y, BUTTONS_WIDTH*0.5f, 20,&fadeSeconds, minFadeSeconds, maxFadeSeconds);
	addControl(fadeSecondsSlider);
	ofAddListener(fadeSecondsSlider->drag, this, &MyVideoPlayerPlus::onFadeSecondsDrag);
	
	fadeToggle = new MyToggle("Fade", BUTTON_PADDING_X*3 + previewW + BUTTONS_WIDTH, 60 + BUTTON_PADDING_Y, BUTTONS_WIDTH*0.5f, 20);
	addControl(fadeToggle);
	fadeToggle->turnOnToggle();
	ofAddListener(fadeToggle->turnOn, this, &MyVideoPlayerPlus::onFadeToggleTurnOn);
	ofAddListener(fadeToggle->turnOff, this, &MyVideoPlayerPlus::onFadeToggleTurnOff);
	
	ofAddListener(vidA->videoLoaded, this, &MyVideoPlayerPlus::onVideoLoaded);
	ofAddListener(vidA->videoError, this, &MyVideoPlayerPlus::onVideoError);
	ofAddListener(vidB->videoLoaded, this, &MyVideoPlayerPlus::onVideoLoaded);
	ofAddListener(vidB->videoError, this, &MyVideoPlayerPlus::onVideoError);
	
	fadeToBlackToggle = new MyToggle("Fade To Black", BUTTON_PADDING_X*3 + previewW + BUTTONS_WIDTH, 145, BUTTONS_WIDTH*0.5f, 50);
	addControl(fadeToBlackToggle);
	fadeToBlackToggle->setStateText("", "");
	fadeToBlackSeconds = 10;
	fadeToBlackSlider = new MySlider("", BUTTON_PADDING_X*3 + previewW + BUTTONS_WIDTH, 195, BUTTONS_WIDTH*0.5f, 20, &fadeToBlackSeconds, 0, 30);
	addControl(fadeToBlackSlider);
	ofAddListener(fadeToBlackToggle->turnOn, this, &MyVideoPlayerPlus::onFadeToBlackTurnOn);
	ofAddListener(fadeToBlackToggle->turnOff, this, &MyVideoPlayerPlus::onFadeToBlackTurnOff);
	ofAddListener(fadeToBlackSlider->drag, this, &MyVideoPlayerPlus::onFadeToBlackSecondsDrag);
	
	fadeToBlack = new FadeToBlack();
	

}

MyVideoPlayerPlus::~MyVideoPlayerPlus(){
	delete vidA;
	delete vidB;
}

void MyVideoPlayerPlus::update(){
	unsigned long currentTime = ofGetSystemTime();
	fadeToBlack->update(currentTime);
	vidA->update(currentTime);
	if(vidA->getAlpha() > 0.0f){
		if((vidA->width != vidATexture->getWidth()) or (vidA->height != vidATexture->getHeight())){
			vidATexture->clear();
			vidATexture->allocate(vidA->width, vidA->height, GL_RGB);
		}
		vidATexture->loadData(vidA->getPixels(), vidA->width, vidA->height, GL_RGB);
	}
	if(vid->bLoaded)videoPosition = vid->getPosition() * videoDuration;
	vidB->update(currentTime);
	if(vidB->getAlpha() > 0.0f){
		if((vidB->width != vidBTexture->getWidth()) or (vidB->height != vidBTexture->getHeight())){
			vidBTexture->clear();
			vidBTexture->allocate(vidB->width, vidB->height, GL_RGB);
		}
		vidBTexture->loadData(vidB->getPixels(), vidB->width, vidB->height, GL_RGB);
	}
	if(vid->bLoaded)videoPosition = vid->getPosition() * videoDuration;
	for(int i=0; i<bLoadThisVideo.size(); i++){
		if(bLoadThisVideo[i] == 1){
			goToVideo(i);
			bLoadThisVideo[i] = 0;
		}
	}
videoList->update();
#ifdef TEST_MODE
	{
		int numIterations = 1;//(int)ofRandom(1, 10);
		for(int i=0; i<numIterations; i++){
			int toggleIndex = (int)ofRandom(0, buttons.size());
			buttons[toggleIndex]->turnOnToggle();
			//waitTillVideoFinishesLoading();
		}
	}
#endif
}

void MyVideoPlayerPlus::draw(){
	MyPanel::draw();
	drawVideo(x + BUTTONS_WIDTH + BUTTON_PADDING_X*2, y + BUTTON_PADDING_Y, previewW, previewH);
	ofNoFill();
	ofSetColor(colors.mainColorPress.r, colors.mainColorPress.g, colors.mainColorPress.b);
	ofRect(x + BUTTONS_WIDTH + BUTTON_PADDING_X*2, y + BUTTON_PADDING_Y, previewW, previewH);
}

void MyVideoPlayerPlus::drawVideo(float _x, float _y, float _w, float _h){
	ofFill();
	if(vidA->getAlpha() > 0.0f){
		glColor4f(1.0f, 1.0f, 1.0f, vidA->getAlpha());
		vidATexture->draw(_x, _y, _w, _h);
	}
	if(vidB->getAlpha() > 0.0f){
		glColor4f(1.0f, 1.0f, 1.0f, vidB->getAlpha());
		vidBTexture->draw(_x, _y, _w, _h);
	}
	if(fadeToBlack->getAlpha() > 0.0f){
		fadeToBlack->draw(_x, _y, _w, _h);
	}
}

void MyVideoPlayerPlus::goToVideo(int vidIndex){
	if(vidIndex == currentVideo)return;
	if(vidIndex >= videoFiles.size())return;
	waitTillVideoFinishesLoading();
	currentVideo = vidIndex;
	if(vid == vidA){
		vidB->close();
		vidB->threadedLoadMovie(videoFiles[currentVideo], false);
	} else {
		vidA->close();
		vidA->threadedLoadMovie(videoFiles[currentVideo], false);
	}
	videoList->selectItem(vidIndex);
}

void MyVideoPlayerPlus::addVideo(string path, string vidName){
	bLoadThisVideo.push_back(0);
	MyVideoPlayer::addVideo(path, vidName);
}

void MyVideoPlayerPlus::resetVideo(){
	goToVideo(0);
	pause();
	fadeToBlack->setAlpha(0.0f);
	fadeToBlack->stopFade();
	if(fadeToBlackToggle->isOn()) fadeToBlackToggle->turnOffToggle();
}

FadableVideo* MyVideoPlayerPlus::getVidA(){
	return vidA;	
}

FadableVideo* MyVideoPlayerPlus::getVidB(){
	return vidB;	
}

ofTexture* MyVideoPlayerPlus::getVidATexture(){
	return vidATexture;
}

ofTexture* MyVideoPlayerPlus::getVidBTexture(){
	return vidBTexture;
}

ofVideoPlayer* MyVideoPlayerPlus::getActiveVideo(){
	return vid;
}

void MyVideoPlayerPlus::useFade(bool _bUseFade){
	if(_bUseFade)fadeToggle->turnOnToggle();
	else fadeToggle->turnOffToggle();
}

bool MyVideoPlayerPlus::useFade(){
	return bUseFade;
}

void MyVideoPlayerPlus::closeAfterFadeOut(bool _bClose){
	bFadeAndClose = _bClose;
}

bool MyVideoPlayerPlus::closeAfterFadeOut(){
	return bFadeAndClose;
}

void MyVideoPlayerPlus::onFadeSecondsDrag(ControlEventArgs& e){
	int _fadeMillis = (int)(fadeSeconds*1000);
	vidA->setFadeMillis(_fadeMillis);
	vidB->setFadeMillis(_fadeMillis);
}

void MyVideoPlayerPlus::onFadeToggleTurnOn(ControlEventArgs& e){
	bUseFade = true;
	int _fadeMillis = (int)(fadeSeconds*1000);
	vidA->setFadeMillis(_fadeMillis);
	vidB->setFadeMillis(_fadeMillis);
}

void MyVideoPlayerPlus::onFadeToggleTurnOff(ControlEventArgs& e){
	bUseFade = false;
	vidA->setFadeMillis(0.0f);
	vidB->setFadeMillis(0.0f);
}

void MyVideoPlayerPlus::onVideoLoaded(FadableVideo*& e){
	if(e == vidA){
		vid = vidA;
		vidA->fadeIn();
		vidB->fadeOutAndClose();
		initVideo(vid);
	} else {
		vid = vidB;
		vidB->fadeIn();
		vidA->fadeOutAndClose();
		initVideo(vid);
	}
	

}

void MyVideoPlayerPlus::onVideoError(FadableVideo*& e){
	goToVideo(currentVideo);
}

void MyVideoPlayerPlus::onVideoSelected(ControlEventArgs& e){
	for(int i=0; i<videoNames.size(); i++){
		if(videoNames[i] == e.control->getName()){
			bLoadThisVideo[i] = 1;
			return;
		}
	}
}
void MyVideoPlayerPlus::onNextReleased(ControlEventArgs& e){
	waitTillVideoFinishesLoading();
	if(isVideoStillLoading())return;
	MyVideoPlayer::onNextReleased(e);
}

void MyVideoPlayerPlus::onPrevReleased(ControlEventArgs& e){
	waitTillVideoFinishesLoading();
	if(isVideoStillLoading())return;
	MyVideoPlayer::onPrevReleased(e);
}

void MyVideoPlayerPlus::onFadeToBlackTurnOn(ControlEventArgs& e){
	fadeToBlack->fadeIn();
}

void MyVideoPlayerPlus::onFadeToBlackTurnOff(ControlEventArgs& e){
	fadeToBlack->fadeOut();
}

void MyVideoPlayerPlus::onFadeToBlackSecondsDrag(ControlEventArgs& e){
	fadeToBlack->setFadeSeconds(fadeToBlackSeconds);
}

void MyVideoPlayerPlus::setFadeColor(float r, float g, float b){
	fadeToBlack->setFadeColor(r, g, b);
}

void MyVideoPlayerPlus::fadeIn(){
	fadeToBlack->fadeIn();
}

void MyVideoPlayerPlus::fadeOut(){
	fadeToBlack->fadeOut();
}

void MyVideoPlayerPlus::waitTillVideoFinishesLoading(){
	vidA->waitTillLoaded();
	vidB->waitTillLoaded();
}

bool MyVideoPlayerPlus::isVideoStillLoading(){
	if(vidA->isLoading()){
		return true;
	}
	if(vidB->isLoading()){
		return true;
	}
	return false;
}


void MyVideoPlayerPlus::loadFromXML(ofxXmlSettings& xml){
	loopToggle->loadFromXML(xml);
	volumeSlider->loadFromXML(xml);
	fadeSecondsSlider->loadFromXML(xml);
	fadeToggle->loadFromXML(xml);
	fadeToBlackSlider->loadFromXML(xml);
}

void MyVideoPlayerPlus::saveToXML(ofxXmlSettings& xml){
	loopToggle->saveToXML(xml);
	volumeSlider->saveToXML(xml);
	fadeSecondsSlider->saveToXML(xml);
	fadeToggle->saveToXML(xml);
	fadeToBlackSlider->saveToXML(xml);
}

#endif






