#pragma once

#include "FadableBase.h"
#include "ofMain.h"

class FadableCam: public FadableBase, public ofVideoGrabber{
protected:
	
public:
	FadableCam(float _w = 320, float _h = 240){
		initGrabber(_w, _h);
	}
	
	virtual ~FadableCam(){
	
	}
	
	virtual void update(unsigned long& currentTime){
		if(!bGrabberInited)return;
		FadableBase::update(currentTime);
		if(alpha > 0.0f)ofVideoGrabber::grabFrame();
	}
	
	virtual void draw(float x, float y){
		if(alpha == 0.0f)return;
		glColor4f(1.0f, 1.0f, 1.0f, alpha);
		ofVideoGrabber::draw(x, y);
	}
	
	virtual void draw(float x, float y, float w, float h){
		if(alpha == 0.0f)return;
		glColor4f(1.0f, 1.0f, 1.0f, alpha);
		ofVideoGrabber::draw(x, y, w, h);
	}
};