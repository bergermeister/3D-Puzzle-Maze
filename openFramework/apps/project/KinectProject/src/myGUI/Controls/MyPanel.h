#pragma once

#include "ControlBase.h"

class MyPanel: public ControlBase{
protected:
	vector<ControlBase*>controls;
public:
	
	MyPanel(string _name, float _x, float _y);
	~MyPanel();
	virtual void clear();
	virtual void update();
	virtual void draw();
	virtual void addControl(ControlBase* c);
	virtual vector<ControlBase*>& getControls();
	
	virtual void setDefaultColors();
	
	virtual void onPress(int x, int y, int button);
	virtual void onRelease(int x, int y, int button);
	virtual void onReleaseOutside(int x, int y, int button);
	virtual void onMouseMove(int x, int y);
	virtual void onRollOut();
	virtual void onDragOver(int x, int y, int button);
	virtual void onDragOutside(int x, int y, int button);
	
	virtual void loadFromXML(ofxXmlSettings& xml);
	virtual void saveToXML(ofxXmlSettings& xml);
};

