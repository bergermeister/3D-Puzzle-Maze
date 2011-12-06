#pragma once

#include "FadableBase.h"
#include "ofMain.h"

class FadeToBlack: public FadableBase{
protected:
	float r,g,b;
public:
	FadeToBlack(){
		r = g = b = 0.0f;
		setAlpha(0.0f);
		stopFade();
	}
	virtual ~FadeToBlack(){
		
	}
	
	virtual void draw(float x, float y, float w, float h){
		if(alpha > 0.0f){
			glColor4f(r, g, b, alpha);
			ofRect(x, y, w, h);
		}
	}
	
	virtual void setFadeColor(float r, float g, float b){
		this->r = r;
		this->g = g;
		this->b = b;
	}
};