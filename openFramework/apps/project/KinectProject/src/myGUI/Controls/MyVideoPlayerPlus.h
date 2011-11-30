#pragma once

#ifdef USE_OFXOPENCV

#include "MyVideoPlayer.h"
#include "FadableVideo.h"
#include "FadableCam.h"
#include "FadeToBlack.h"

//#define TEST_MODE

class MySlider;
class MyButton;
class MyToggle;

class MyVideoPlayerPlus:public MyVideoPlayer{
protected:
	
	FadableVideo* vidA, *vidB;
	ofTexture* vidATexture, *vidBTexture;
	FadableVideo* whichVideo;
	FadeToBlack* fadeToBlack;
	MyToggle *fadeToBlackToggle;
	MySlider* fadeToBlackSlider;
	float fadeToBlackSeconds;
	bool bFadeAndClose;
	float fadeSeconds;
	bool bUseFade;
	
	bool bResetVideo;
	
	
	vector<int>bLoadThisVideo;
	
	virtual void onFadeSecondsDrag(ControlEventArgs& e);
	virtual void onFadeToggleTurnOn(ControlEventArgs& e);
	virtual void onFadeToggleTurnOff(ControlEventArgs& e);
	
	virtual void onVideoLoaded(FadableVideo*& e);
	virtual void onVideoError(FadableVideo*& e);
	
	virtual void onNextReleased(ControlEventArgs& e);
	virtual void onPrevReleased(ControlEventArgs& e);
	
	virtual void onVideoSelected(ControlEventArgs& e);
	
	virtual void onFadeToBlackTurnOn(ControlEventArgs& e);
	virtual void onFadeToBlackTurnOff(ControlEventArgs& e);
	virtual void onFadeToBlackSecondsDrag(ControlEventArgs& e);
	
	virtual void waitTillVideoFinishesLoading();
	virtual bool isVideoStillLoading();

public:
	
	MySlider* fadeSecondsSlider;
	MyToggle* fadeToggle;
	
	MyVideoPlayerPlus(string _name, float _x, float _y, float _w, float _h);
	~MyVideoPlayerPlus();
	
	virtual void update();
	virtual void draw();
	virtual void drawVideo(float _x, float _y, float _w, float _h);
	virtual void goToVideo(int vidIndex);
	
	virtual void addVideo(string path, string vidName);
	
	virtual void resetVideo();
	
	virtual FadableVideo* getVidA();
	virtual FadableVideo* getVidB();
	
	virtual ofTexture* getVidATexture();
	virtual ofTexture* getVidBTexture();
	
	virtual ofVideoPlayer* getActiveVideo();
	
	virtual void useFade(bool _bUseFade);
	virtual bool useFade();
	
	virtual void closeAfterFadeOut(bool _bClose);
	virtual bool closeAfterFadeOut();
	
	virtual void loadFromXML(ofxXmlSettings& xml);
	virtual void saveToXML(ofxXmlSettings& xml);
	
	virtual void setFadeColor(float r, float g, float b);
	virtual void fadeIn();
	virtual void fadeOut();
	
};

#endif
