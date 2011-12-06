#pragma once

#include "ControlBase.h"

#define INFO_MODE_VALUE 0
#define INFO_MODE_TIME 1

class MySlider: public ControlBase{
protected:
	virtual void onPress(int x, int y, int button);
	virtual void onRelease(int x, int y, int button);
	
	virtual void onDragOver(int _x, int _y, int _button);
	virtual void onDragOutside(int _x, int _y, int _button);
	
	
	float* valFloatPtr;
	int* valIntPtr;
	float normalVal;
	float maxVal, minVal;
	
	bool bIsInt;
	
	unsigned char infoMode;
	
	void calcNormalVal();
	void calcVal();
	
public:
	MySlider(string _name, float _x, float _y, float _w, float _h, float*_val, float _minVal, float _maxVal);
	MySlider(string _name, float _x, float _y, float _w, float _h, int*_val, float _minVal, float _maxVal);
	~MySlider();
	
	virtual void draw();
	
	virtual void setMaxVal(float max);
	virtual void setMinVal(float min);
	
	virtual void setDefaultColors();
	virtual bool hasIntVal();
	virtual bool hasFloatVal();
	
	virtual float getVal();
	
	virtual void setInfoMode(unsigned char _mode = INFO_MODE_VALUE);
	
	virtual void loadFromXML(ofxXmlSettings& xml);
	virtual void saveToXML(ofxXmlSettings& xml);
	
	ofEvent<ControlEventArgs>drag;
};
