#pragma once

#ifdef USE_OFXOPENCV

#include "ofxOpenCv.h"
#include "FadableBase.h"

class FadableCvGrayImage: public ofxCvGrayscaleImage, public FadableBase{
protected:
	
public:
	FadableCvGrayImage(){}
	virtual ~FadableCvGrayImage(){}
	
	virtual void draw(float x, float y){
		glColor4f(1.0f, 1.0f, 1.0f, alpha);
		ofxCvGrayscaleImage::draw(x, y);
	}
	virtual void draw(float x, float y, float w, float h){
		glColor4f(1.0f, 1.0f, 1.0f, alpha);
		ofxCvGrayscaleImage::draw(x, y, w, h);
	}
	
	virtual void drawWithFullAlpha(float x, float y, float w, float h){
		glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
		ofxCvGrayscaleImage::draw(x, y, w, h);
	}
};

#endif
