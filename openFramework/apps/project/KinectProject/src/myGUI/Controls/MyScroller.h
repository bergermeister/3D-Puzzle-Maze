#pragma once

#include "ControlBase.h"
#include "ofMain.h"

class MyButton;

class MyScroller: public ControlBase{
protected:
	float oldScrollerPosY, mouseOffSetY;
	float positionNorm;
	ofRectangle dragRect;
	bool bDrag;

public:
	MyScroller(float _x, float _y, float _w, float _h);
	~MyScroller();
	
	virtual void update();
	virtual void draw();
	virtual float getPosition();
	
	virtual void onPress(int x, int y, int button);
	virtual void onRelease(int x, int y, int button);
	virtual void onReleaseOutside(int x, int y, int button);
	virtual void setDefaultColors();
	virtual void adjustVertices();
	virtual void setPosition(float _posNorm);
	ofEvent<ControlEventArgs> drag;
};

