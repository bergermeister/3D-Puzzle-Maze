

#include "MySlider2d.h"

MySlider2d::MySlider2d(string _name, float _x, float _y, float _w, float _h, ofPoint* _val, ofPoint _min, ofPoint _max):
ControlBase(_name)
{
	type = CONTROL_TYPE_SLIDER_2D;
	setPos(_x, _y);
	setSize(_w, _h);
	adjustVertices();
	val = _val;
	min = _min;
	max = _max;
	enable();
	calcNormalVal();
	setDefaultColors();
}

MySlider2d::~MySlider2d(){
	
}

void MySlider2d::draw(){
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
	drawText();
	calcNormalVal();
	float yPos = normalVal.y * height + y;
	ofLine(x, yPos, x + width, yPos);
	float xPos = normalVal.x * width + x;
	ofLine(xPos, y, xPos, y + height);
	ofNoFill();
	ofRect(x, y, width, height);
}

void MySlider2d::calcNormalVal(){
	normalVal.x = (val->x - min.x) / (max.x - min.x);
	normalVal.y = (val->y - min.y) / (max.y - min.y);
}

void MySlider2d::calcVal(){
	val->x = MIN(max.x, MAX(min.x, (normalVal.x * (max.x - min.x)) + min.x));
	val->y = MIN(max.y, MAX(min.y, (normalVal.y * (max.y - min.y)) + min.y));

	
}

void MySlider2d::setDefaultColors(){
	colors.mainColorPress = colors.mainColorOver;
}

void MySlider2d::onPress(int _x, int _y, int _button){
	normalVal.x = (_x - x) / width;
	normalVal.y = (_y - y) / height;
	calcVal();
	ControlBase::onPress(_x, _y, _button);
	ControlEventArgs e(this);
	ofNotifyEvent(drag , e);
}

void MySlider2d::onRelease(int _x, int _y, int _button){
	normalVal.x = (_x - x) / width;
	normalVal.y = (_y - y) / height;
	calcVal();
	ControlBase::onRelease(_x, _y, _button);
}

void MySlider2d::onDragOver(int _x, int _y, int _button){
	onPress(_x, _y, _button);
	ControlEventArgs e(this);
	ofNotifyEvent(drag, e);
}

void MySlider2d::onDragOutside(int _x, int _y, int _button){
	onPress(_x, _y, _button);
}

void MySlider2d::loadFromXML(ofxXmlSettings& xml){
	string tag = generateXMLTag();
	val->x = xml.getValue(tag + ":" + "xVal", 0.0);
	val->y = xml.getValue(tag + ":" + "yVal", 0.0);
}

void MySlider2d::saveToXML(ofxXmlSettings& xml){
	string tag = generateXMLTag();
	xml.addTag(tag);
	xml.pushTag(tag);
	xml.addValue("xVal", val->x);
	xml.addValue("yVal", val->y);
	xml.popTag();
}

