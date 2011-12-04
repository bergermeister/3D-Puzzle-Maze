/*
 *  MyToggle.cpp
 *  MyGUI_002
 *
 *  Created by rui madeira on 8/17/09.
 *  Copyright 2009 LISBONlab. All rights reserved.
 *
 */

#include "MyToggle.h"

MyToggle::MyToggle(string _name, float _x, float _y, float _w, float _h):
ControlBase(_name)
{
	type = CONTROL_TYPE_TOGGLE;
	setPos(_x, _y);
	setSize(_w, _h);
	adjustVertices();
	setDefaultColors();
	setStateText();
	bIsOn = false;
}

MyToggle::~MyToggle(){
	
}

void MyToggle::onPress(int x, int y, int button){
	bPressed = true;
}

void MyToggle::onRelease(int x, int y, int button){
	if(!bPressed){
		if(controlState == CONTROL_STATE_IDLE) controlState = CONTROL_STATE_OVER;
		return;
	}
	ControlEventArgs e(this);
	bPressed = false;
	if(controlState == CONTROL_STATE_OVER){
		turnOnToggle();
	} else if(controlState == CONTROL_STATE_PRESS){
		turnOffToggle();
	}
	ofNotifyEvent(release, e);
}

void MyToggle::onReleaseOutside(int x, int y, int button){
	
}

void MyToggle::onDragOver(int x, int y, int button){
	
}

void MyToggle::onDragOutside(int x, int y, int button){
	
}

void MyToggle::onRollOut(){
	if(!bIsOn){
		controlState = CONTROL_STATE_IDLE;
	}
}

void MyToggle::onRollOver(int x, int y){
	if(!bIsOn){
		controlState = CONTROL_STATE_OVER;
	}
}

void MyToggle::setStateText(string _onText, string _offText){
	setOnText(_onText);
	setOffText(_offText);
}

void MyToggle::setOnText(string t){
	onText = t;
}

void MyToggle::setOffText(string t){
	offText = t;
}

void MyToggle::drawText(){
	ofSetColor(colors.textColor.r, colors.textColor.g, colors.textColor.b);
	if(onText.size() > 0 || offText.size() > 0){
	string theText;
	if(bIsOn){
		theText = name + ": " + onText;
	} else {
		theText = name + ": " + offText;	
	}
	ofDrawBitmapString(theText, x + textOffSet.x, y + textOffSet.y);
	} else ofDrawBitmapString(name, x + textOffSet.x, y + textOffSet.y);
}

void MyToggle::turnOnToggle(){
	if(isOn())return;
	bIsOn = true;
	controlState = CONTROL_STATE_PRESS;
	ControlEventArgs e(this);
	ofNotifyEvent(turnOn, e);
}

void MyToggle::turnOffToggle(){
	if(isOff())return;
	bIsOn = false;
	if(hitTest(_mouseX, _mouseY)){
		controlState = CONTROL_STATE_OVER;
	} else {
		controlState = CONTROL_STATE_IDLE;
	}
	ControlEventArgs e(this);
	ofNotifyEvent(turnOff, e);
}

bool MyToggle::isOn(){
	return bIsOn;	
}

bool MyToggle::isOff(){
	return !bIsOn;
}

void MyToggle::loadFromXML(ofxXmlSettings& xml){
	string tag = generateXMLTag();
	if(xml.getValue(tag + ":" + "isOn", false)){
		turnOnToggle();
	} else turnOffToggle();	
}

void MyToggle::saveToXML(ofxXmlSettings& xml){
	string tag = generateXMLTag();
	xml.addValue(tag + ":" + "isOn", isOn());
}

