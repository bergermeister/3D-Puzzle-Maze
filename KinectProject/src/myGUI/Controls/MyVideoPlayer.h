#pragma once

#ifdef USE_OFXOPENCV

#include "MyRadioButtons.h"
#include "ControlEvent.h"
#include "ofxDirList.h"
#include "ofxOpenCv.h"

#define BUTTONS_WIDTH 250
#define BUTTONS_HEIGHT 50
#define BUTTON_PADDING_X 10
#define BUTTON_PADDING_Y 30

#define MAX_VOLUME 100

class MyButton;
class MySlider;
class MyScrollList;

class MyVideoPlayer: public MyPanel{
protected:
	vector<string>videoFiles, videoNames;
	float previewW, previewH;
	ofVideoPlayer* vid;
	int currentVideo;
	float videoPosition;
	float videoDuration;
	bool bPaused;
	MyButton* prevButton, *nextButton;
	MyToggle* playToggle, *loopToggle;
	MySlider* videoPosSlider, *volumeSlider;
	MyScrollList* videoList;
	float volumeVal;
	int bLoop;
	
public:
	MyVideoPlayer(string _name, float _x, float _y, float _w, float _h);
	virtual ~MyVideoPlayer();
	
	virtual void update();
	virtual void draw();
	virtual void drawVideo(float _x, float _y, float _w, float _h);
	virtual void addVideo(string path, string vidName);
	virtual void loadVideosFromFolder(string path);
	virtual void play();
	virtual void pause();
	virtual void next();
	virtual void prev();
	virtual void goToVideo(int vidIndex);
	virtual void goToVideo(string videoName);
	virtual void setVolume(float _val);
	virtual void enable();
	virtual void disable();
	virtual void setListHeight(float height);
	
	virtual void setLoop(bool _bLoop = true);
	virtual ofVideoPlayer* getVideo();
	
	virtual void loadFromXML(ofxXmlSettings& xml);
	virtual void saveToXML(ofxXmlSettings& xml);
	
	virtual int getCurrentVideoIndex();
	virtual string getCurrentVideoName();
	virtual string getCurrentVideoPath();
	virtual int getNumVideos();
	
protected:
	//internal events
	virtual void onPlayTurnOn(ControlEventArgs& e);
	virtual void onPlayTurnOff(ControlEventArgs& e);
	
	virtual void onNextReleased(ControlEventArgs& e);
	virtual void onPrevReleased(ControlEventArgs& e);
	
	virtual void onVideoSelected(ControlEventArgs& e);
								 
	virtual void onVidPosDrag(ControlEventArgs& e);
	
	virtual void onLoopTurnOn(ControlEventArgs& e);
	virtual void onLoopTurnOff(ControlEventArgs& e);
	
	virtual void onVolumeDrag(ControlEventArgs& e);
	
	virtual void addControlEvents();
	virtual void removeControlEvents();
	
	virtual void addMovieEvents();
	virtual void removeMovieEvents();
	
	virtual void addEvents();
	virtual void removeEvents();
	
	virtual void initVideo(ofVideoPlayer* _vid);
};

#endif

