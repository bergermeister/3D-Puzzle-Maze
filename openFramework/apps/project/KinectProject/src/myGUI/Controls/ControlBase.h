#pragma once

#include "ofxMSAInteractiveObject.h"
#include "ofMain.h"
#include "ControlColorScheme.h"
#include "ControlEvent.h"
#include "ofxXmlSettings.h"

#define CONTROL_TYPE_BUTTON 0
#define CONTROL_TYPE_SLIDER 1
#define CONTROL_TYPE_PANEL 2
#define CONTROL_TYPE_TOGGLE 3
#define CONTROL_TYPE_RADIO_BUTTONS 4
#define CONTROL_TYPE_VIDEO_PLAYER 5
#define CONTROL_TYPE_VIDEO_PLAYER_PLUS 6
#define CONTROL_TYPE_SLIDER_2D 7
#define CONTROL_TYPE_SCROLL_LIST 8
#define CONTROL_TYPE_SCROLLER 9

#define CONTROL_STATE_IDLE 0
#define CONTROL_STATE_OVER 1
#define CONTROL_STATE_PRESS 2



class MyGUI;

class ControlBase: public ofxMSAInteractiveObject{
protected:
	static int totalControlsCreated;
	int controlId;
	ControlColorScheme colors;
	string name;
	//unsigned int controlId;
	unsigned int type;
	unsigned char controlState;
	
	vector<float>vertices;
	bool bPressed;
	
	ofPoint textOffSet;
	
	friend class MyGUI;
	
	bool bEnabled;
	
public:
	ControlBase(string _name);
	
	virtual ~ControlBase();
	
	virtual unsigned int getType();	
	virtual void setName(string _name);	
	virtual string getName();
	virtual bool isButton();	
	virtual bool isSlider();	
	virtual void setControlId(unsigned int _id);	
	virtual void adjustVertices();	
	virtual void setState(unsigned char _state);
	virtual bool isEnabled();
	virtual bool isDisabled();
	virtual void enable();
	virtual void disable();
	virtual unsigned char getState();
	virtual void onPress(int x, int y, int button);	
	virtual void onRelease(int x, int y, int button);	
	virtual void onReleaseOutside(int x, int y, int button);	
	virtual void onRollOver(int x, int y);	
	virtual void onRollOut();	
	virtual void onDragOver(int x, int y, int button);	
	virtual void onDragOutside(int x, int y, int button);
	virtual void drawFill();	
	virtual void drawOutline();	
	virtual void drawText();
	virtual void draw();	
	virtual void update(){}
	virtual ControlColorScheme& getColorScheme();	
	virtual void setDefaultColors();
	virtual void loadFromXML(ofxXmlSettings& xml){}
	virtual void saveToXML(ofxXmlSettings& xml){}
	virtual string generateXMLTag();
	virtual bool isPointInside(const ofPoint& pt);
	
	ofEvent<ControlEventArgs>press;
	ofEvent<ControlEventArgs>release;
	ofEvent<ControlEventArgs>rollOver;
	ofEvent<ControlEventArgs>rollOut;
};
