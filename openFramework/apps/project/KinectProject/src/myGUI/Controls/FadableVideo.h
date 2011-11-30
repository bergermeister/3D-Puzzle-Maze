#pragma once

#include "FadableBase.h"
#include "ofMain.h"
#include "ofxRuiThread.h"

#define ACTION_AFTER_FADE_OUT_CLOSE 0
#define ACTION_AFTER_FADE_OUT_PAUSE 1
#define ACTION_AFTER_FADE_OUT_REWIND 2

#define DEFAULT_ACTION ACTION_AFTER_FADE_OUT_CLOSE

class FadableVideo: public FadableBase, public ofVideoPlayer, public ofxRuiThread{
protected:
	unsigned char actionAfterFadeOut;
	float oldAlpha;
	string fileToLoad;
	bool bIsLoading;
	bool bBeginLoad;
public:

	ofEvent<FadableVideo*>videoLoaded;
	ofEvent<FadableVideo*>videoError;

	FadableVideo(){
		fileToLoad = "";
		bPaused = true;
		blocking = true;
		bIsLoading = false;
		bBeginLoad = false;
		verbose = false;
	}

	~FadableVideo(){

	}

	virtual void threadedLoadMovie(string _file, bool _bVerbose = false){
		if(isLoading())return;
		lock();
		fileToLoad = _file;
		bIsLoading = true;
		bBeginLoad = true;
		unlock();
		startThread(true, false);
		
	
	}
	
	void threadedFunction(){
		lock();
		loadMovie(fileToLoad);
		stopThread();
		bIsLoading = false;
		unlock();
	}
	

	virtual bool isLoading(){
		blocking = false;
		bool _bLoading = !lock();
		if(!_bLoading) unlock();
		blocking = true;
		return _bLoading;
	}

	virtual void waitTillLoaded(){
		blocking = false;
		while(isLoading()){}
	}

	virtual void update(unsigned long& currentTime){
		blocking = false;
		if(lock()){
			if(bBeginLoad){
				if(!bIsLoading){
					bBeginLoad = false;
					FadableVideo* ptr = this; //using "this" directly causes an error
					if(bLoaded)ofNotifyEvent(videoLoaded, ptr, this);
					else ofNotifyEvent(videoError, ptr, this);
				}
			}
			unlock();
		}
		blocking = true;
		
		FadableBase::update(currentTime);
		if(oldAlpha > 0.0f and alpha == 0.0f){
			switch(actionAfterFadeOut){
				case ACTION_AFTER_FADE_OUT_CLOSE:
					close();
					break;
				case ACTION_AFTER_FADE_OUT_PAUSE:
					setPaused(true);
					break;
				case ACTION_AFTER_FADE_OUT_REWIND:
					setPosition(0.0f);
					break;
			}
		} else if(oldAlpha == 0.0f and alpha > 0.0f){
			setPaused(bPaused);
		}
		if(alpha > 0.0f){
			ofVideoPlayer::update();
		}
		//if(bFadeIn or bFadeOut)setVolume(alpha*100);
		oldAlpha = alpha;
	}

	virtual void draw(float x, float y, float w, float h){
		if(alpha == 0.0f) return;
		glColor4f(1.0f, 1.0f, 1.0f, alpha);
		ofVideoPlayer::draw(x, y, w, h);
	}

	virtual void draw(float x, float y){
		if(alpha == 0.0f)return;
		glColor4f(1.0f, 1.0f, 1.0f, alpha);
		ofVideoPlayer::draw(x, y);
	}

	virtual void fadeOutAndClose(){
		actionAfterFadeOut = ACTION_AFTER_FADE_OUT_CLOSE;
		fadeOut();
	}

	virtual void fadeOutAndPause(){
		actionAfterFadeOut = ACTION_AFTER_FADE_OUT_PAUSE;
		fadeOut();

	}

	virtual void fadeOutAndRewind(){
		actionAfterFadeOut = ACTION_AFTER_FADE_OUT_REWIND;
		fadeOut();
	}

/*	virtual bool blockLock(){
		if(blocking){
			lock();
		} else {
			blocking = true;
			lock();
			blocking = false;
		}
		return true;
	}

	virtual bool tryLock(){
		if(blocking){
			blocking = false;
			bool bLocked = lock();
			blocking = true;
			return bLocked;
		} else {
			return lock();
		}
	}*/
};

