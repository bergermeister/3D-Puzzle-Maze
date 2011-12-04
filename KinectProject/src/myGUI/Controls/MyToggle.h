#pragma once

#include "ControlBase.h"

class MyToggle:public ControlBase{
protected:
	bool* val;
	bool bIsOn;
	string onText, offText;
public:
	
	MyToggle(string _name, float _x, float _y, float _w, float _h);
	~MyToggle();

	virtual void onRelease(int x, int y, int button);
	virtual void onReleaseOutside(int x, int y, int button);
	virtual void onDragOver(int x, int y, int button);
	virtual void onDragOutside(int x,int y , int button);
	virtual void onPress(int x, int y, int button);
	virtual void onRollOut();
	virtual void onRollOver(int x, int y);
	virtual void setStateText(string _onText = "ON", string _offText = "OFF");
	virtual void setOnText(string t);
	virtual void setOffText(string t);
	virtual bool isOn();
	virtual bool isOff();
	
	virtual void turnOnToggle();
	virtual void turnOffToggle();
	
	virtual void drawText();
	
	virtual void saveToXML(ofxXmlSettings& xml);
	virtual void loadFromXML(ofxXmlSettings& xml);
	
	ofEvent<ControlEventArgs> turnOn, turnOff;
};

