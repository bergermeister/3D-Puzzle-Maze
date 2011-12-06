
#include "MySlider.h"

MySlider::MySlider(string _name, float _x, float _y, float _w, float _h, float* _val, float _minVal, float _maxVal):
ControlBase(_name)
{
	type = CONTROL_TYPE_SLIDER;
	setPos(_x, _y);
	setSize(_w, _h);
	adjustVertices();
	valFloatPtr = _val;
	setMaxVal(_maxVal);
	setMinVal(_minVal);
	bIsInt = false;
	*valFloatPtr = MIN(maxVal, MAX(minVal, *valFloatPtr)); 
	calcNormalVal();
	setDefaultColors();
	setInfoMode();
}

MySlider::MySlider(string _name, float _x, float _y, float _w, float _h, int* _val, float _minVal, float _maxVal):
ControlBase(_name)
{
	type = CONTROL_TYPE_SLIDER;
	setPos(_x, _y);
	setSize(_w, _h);
	adjustVertices();
	valIntPtr = _val;
	setMaxVal(_maxVal);
	setMinVal(_minVal);
	bIsInt = true;
	*valIntPtr = MIN(maxVal, MAX(minVal, *valIntPtr));
	calcNormalVal();	
	setDefaultColors();
	setInfoMode();
}

MySlider::~MySlider(){
	
}

void MySlider::draw(){
	drawFill();
	switch(controlState){
		case CONTROL_STATE_IDLE:
			ofSetColor(colors.secColorIdle.r, colors.secColorIdle.g, colors.secColorIdle.b);
			break;
		case CONTROL_STATE_OVER:
			ofSetColor(colors.secColorOver.r, colors.secColorOver.g, colors.secColorOver.b);
			break;
		case CONTROL_STATE_PRESS:
			ofSetColor(colors.secColorPress.r, colors.secColorPress.g, colors.secColorPress.b);
			break;
	}
	ofFill();
	if(bIsInt){
		int& intValRef = *valIntPtr;
		if(intValRef < minVal) intValRef = minVal;
		if(intValRef > maxVal) intValRef = maxVal;
	} else {
		
	}
	calcNormalVal();
	float drawWidth = MAX(0, width * normalVal);
	
	ofRect(x, y, drawWidth, height);
	drawOutline();
	ofSetColor(colors.textColor.r, colors.textColor.g, colors.textColor.b);
	string infoText;
	if(name.size() > 0) infoText = name + ": ";
	else infoText = "";
	switch(infoMode){
		case INFO_MODE_VALUE:
			if(bIsInt) infoText = infoText + ofToString(*valIntPtr, 0);
			else infoText = infoText + ofToString(*valFloatPtr, 3);
			break;
		case INFO_MODE_TIME:
			int hours, mins, secs;
			string hoursStr, minsStr, secsStr;
			float inv60 = 1.0f / 60.0f;
			if(bIsInt){
				secs = *valIntPtr;
			} else {
				secs = (int)(*valFloatPtr);
			}
			mins = secs * inv60;
			hours = mins * inv60;
			secs = secs % 60;
			mins = mins % 60;
			hoursStr = ofToString(hours, 0);
			minsStr = ofToString(mins, 0);
			secsStr = ofToString(secs, 0);
			if(hoursStr.size() == 1) hoursStr = "0" + hoursStr;
			if(minsStr.size() == 1) minsStr = "0" + minsStr;
			if(secsStr.size() == 1) secsStr = "0" + secsStr;
			
			infoText = infoText + hoursStr + ":" + minsStr + ":" + secsStr;
			break;
	}
	ofDrawBitmapString(infoText,  x + textOffSet.x, y + textOffSet.y);
}

void MySlider::setMaxVal(float max){
	maxVal = max;
}

void MySlider::setMinVal(float min){
	minVal = min;	
}


void MySlider::onPress(int _x, int _y, int _button){
	normalVal = (_x - x) / width;
	calcVal();
	ControlBase::onPress(_x, _y, _button);
	ControlEventArgs e(this);
	ofNotifyEvent(drag, e);
}

void MySlider::onRelease(int _x, int _y, int _button){
	normalVal = (_x - x) / width;
	calcVal();
	ControlBase::onRelease(_x, _y, _button);
}

void MySlider::onDragOver(int _x, int _y, int _button){
	onPress(_x, _y, _button);
	ControlEventArgs e(this);
	ofNotifyEvent(drag, e);
}

void MySlider::onDragOutside(int _x, int _y, int _button){
	if(_x > x + width){
		if(bIsInt) *valIntPtr = maxVal;
		else *valFloatPtr = maxVal;
		ControlBase::onPress(_x, _y, _button);
	} else if(_x < x){
		if(bIsInt)*valIntPtr = minVal;
		else *valFloatPtr = minVal;
		ControlBase::onPress(_x, _y, _button);
	} else {
		onPress(_x, _y, _button);
	}
	calcNormalVal();
	ControlEventArgs e(this);
	ofNotifyEvent(drag, e);
}

void MySlider::setDefaultColors(){
	colors.mainColorPress = colors.mainColorOver;
	
	colors.secColorIdle.r = 100;
	colors.secColorIdle.g = 100;
	colors.secColorIdle.b = 100;
	
	colors.secColorOver.r = 120;
	colors.secColorOver.g = 130;
	colors.secColorOver.b = 120;
	
	colors.secColorPress.r = 150;
	colors.secColorPress.g = 160;
	colors.secColorPress.b = 150;
	
	colors.outlineColor = colors.secColorPress;
}

void MySlider::calcNormalVal(){
	if(maxVal == minVal){
		normalVal = 0; 
		return;
	}
	if(bIsInt){
		normalVal = ((float)(*valIntPtr) - minVal) / (maxVal - minVal);
	} else {
		normalVal = ((*valFloatPtr) - minVal) / (maxVal - minVal);
	}
}

void MySlider::calcVal(){
	if(bIsInt){
		(*valIntPtr) = (int)((normalVal * (maxVal-minVal)) + minVal);
	} else {
		(*valFloatPtr) = (normalVal * (maxVal-minVal)) + minVal;
	}
}

bool MySlider::hasIntVal(){
	return bIsInt;
}

bool MySlider::hasFloatVal(){
	return !bIsInt;	
}

float MySlider::getVal(){
	if(bIsInt) return (float)(*valIntPtr);
	return *valFloatPtr;
}

void MySlider::setInfoMode(unsigned char _infoMode){
	infoMode = _infoMode;
}

void MySlider::loadFromXML(ofxXmlSettings& xml){
	string tag = generateXMLTag();
	if(bIsInt) *valIntPtr = xml.getValue(tag + ":" + "val", 0);
	else *valFloatPtr = xml.getValue(tag + ":" + "val", 0.0);
}

void MySlider::saveToXML(ofxXmlSettings& xml){
	string tag = generateXMLTag();
	if(bIsInt)xml.addValue(tag + ":" + "val", *valIntPtr);
	else xml.addValue(tag + ":" + "val", *valFloatPtr);
}
