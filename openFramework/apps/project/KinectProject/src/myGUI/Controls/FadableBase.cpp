

#include "FadableBase.h"

FadableBase::FadableBase(){
	bFadeIn = false;
	bFadeOut = false;
	bBeginFade = false;
	alpha = 1.0f;
	fadeMillis = DEFAULT_FADE_MILLIS;
	timeStartFade = timeEndFade = 0L;
}

FadableBase::~FadableBase(){
	
}

void FadableBase::update(unsigned long& currentTime){
	if(bBeginFade){
		bBeginFade = false;
		timeStartFade = currentTime;
		if(bFadeIn)timeEndFade = currentTime + (unsigned long)((1.0f - alpha) * fadeMillis);
		else timeEndFade = currentTime + (unsigned long)(alpha*fadeMillis);
		if(timeEndFade == currentTime){
			if(bFadeIn){
				bFadeIn = false;
				alpha = 1.0f;
			} else {
				bFadeOut = false;
				alpha = 0.0f;
			}
		}
	}
	if(bFadeIn){
		alpha = 1.0f - ((float)(timeEndFade - currentTime)) / (float)fadeMillis;
		if(currentTime > timeEndFade){
			bFadeIn = false;
			alpha = 1.0f;
		}
	} else if(bFadeOut){
		alpha = (float)(timeEndFade - currentTime) / (float)fadeMillis;
		if(currentTime > timeEndFade){
			bFadeIn = false;
			alpha = 0.0f;
		}
	}
}

void FadableBase::fadeIn(){
	if(bFadeIn)return;
	if(alpha == 1.0f) return;
	bBeginFade = true;
	bFadeIn = true;
	bFadeOut = false;
}

void FadableBase::fadeOut(){
	if(bFadeOut)return;
	if(alpha == 0.0f) return;
	bBeginFade = true;
	bFadeOut = true;
	bFadeIn = false;
}

void FadableBase::setFadeSeconds(int _seconds){
	setFadeMillis(_seconds * 1000);
}

void FadableBase::setFadeMillis(int _millis){
	fadeMillis = _millis;
}

void FadableBase::setAlpha(float _a){
	alpha = _a;
}

float FadableBase::getAlpha(){
	return alpha;
}

void FadableBase::stopFade(){
	bBeginFade = bFadeIn = bFadeOut = false;
}

bool FadableBase::isFading(){
	return alpha != 0.0f && alpha != 1.0f;
}

bool FadableBase::isFadingIn(){
	return bFadeIn;
}

bool FadableBase::isFadingOut(){
	return bFadeOut;	
}




