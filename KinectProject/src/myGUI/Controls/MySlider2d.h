#pragma once

#include "ControlBase.h"
#include "ofMain.h"

class MySlider2d: public ControlBase{
protected:
	ofPoint * val;
	ofPoint min, max, normalVal;
	
	virtual void calcNormalVal();
	virtual void calcVal();
	virtual void setDefaultColors();
	
	virtual void onPress(int x, int y, int button);
	virtual void onRelease(int x, int y, int button);
	
	virtual void onDragOver(int _x, int _y, int _button);
	virtual void onDragOutside(int _x, int _y, int _button);
public:
	MySlider2d(string _name, float _x, float _y, float _w, float _h, ofPoint* _val, ofPoint _min, ofPoint _max);
	virtual ~MySlider2d();
	
	virtual void draw();
	virtual void loadFromXML(ofxXmlSettings& xml);
	virtual void saveToXML(ofxXmlSettings& xml);
	
	ofEvent<ControlEventArgs>drag;
};

