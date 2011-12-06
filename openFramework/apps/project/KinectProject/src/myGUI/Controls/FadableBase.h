#pragma once

#include "ofMain.h"


#define DEFAULT_FADE_MILLIS 1000

class FadableBase{
//protected:
public:
	float alpha;
	int fadeMillis;
	unsigned long timeStartFade, timeEndFade;
	bool bBeginFade, bFadeIn, bFadeOut;
public:
	
	FadableBase();
	virtual ~FadableBase();
	
	virtual void update(unsigned long& currentTime);
	virtual void fadeIn();
	virtual void fadeOut();
	virtual void setFadeSeconds(int _seconds);
	virtual void setFadeMillis(int _millis);
	virtual void setAlpha(float _a);
	virtual float getAlpha();
	virtual void stopFade();
	bool isFading();
	bool isFadingIn();
	bool isFadingOut();
};

