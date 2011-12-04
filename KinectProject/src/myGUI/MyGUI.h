#pragma once

//#define USE_OFXOPENCV

#include "ofMain.h"
#include "Controls/ControlIncludes.h"

class MyGUI{
protected:
	vector<ControlBase*>controls;
	bool bEnabled;
	void update(ofEventArgs &e);
	void mouseMoved(ofMouseEventArgs &e);
	void mousePressed(ofMouseEventArgs &e);
	void mouseDragged(ofMouseEventArgs &e);
	void mouseReleased(ofMouseEventArgs &e);
	void keyPressed(ofKeyEventArgs &e);
	void keyReleased(ofKeyEventArgs &e);
	ofxXmlSettings xml;
	string xmlFile;
	
public:
	MyGUI();
	~MyGUI();
	
	void addControl(ControlBase* control);

	void draw();
	void enable();
	void disable();
	void toggle();
	bool isEnabled();
	bool isDisabled();
	
	void setXMLFile(string file = "guiSettings.xml");
	void saveToXML();
	void loadFromXML();

	bool isPointInControl(const ofPoint& pt);
};