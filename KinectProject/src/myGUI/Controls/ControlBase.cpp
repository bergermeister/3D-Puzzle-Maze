#include "ControlBase.h"

int ControlBase::totalControlsCreated = 0;

ControlBase::ControlBase(string _name){
	totalControlsCreated++;
	controlId = totalControlsCreated;
	disableAllEvents();
	vertices.resize(10);
	bPressed = false;
	controlState = CONTROL_STATE_IDLE;
	textOffSet.set(10, 15);
	name = _name;
	enable();
}

ControlBase::~ControlBase(){
	vertices.clear();
}

unsigned int ControlBase::getType(){
	return type;
}

void ControlBase::setName(string _name){
	name = _name;
}

string ControlBase::getName(){
	return name;	
}

bool ControlBase::isButton(){
	return type == CONTROL_TYPE_BUTTON;
}

bool ControlBase::isSlider(){
	return type == CONTROL_TYPE_SLIDER; 
}

void ControlBase::setControlId(unsigned int _id){
	controlId = _id;
}

void ControlBase::setState(unsigned char _state){
	controlState = _state;	
}

bool ControlBase::isEnabled(){
	return bEnabled;	
}

bool ControlBase::isDisabled(){
	return !bEnabled;	
}

void ControlBase::enable(){
	bEnabled = true;
}

void ControlBase::disable(){
	bEnabled = false;
}


unsigned char ControlBase::getState(){
	return controlState;	
}

void ControlBase::adjustVertices(){
	vertices[0] = x;
	vertices[1] = y;
	vertices[2] = x + width;
	vertices[3] = y;
	vertices[4] = x + width;
	vertices[5] = y + height;
	vertices[6] = x;
	vertices[7] = y + height;
	vertices[8] = x;
	vertices[9] = y;
}

void ControlBase::onPress(int x, int y, int button){
	if(isDisabled())return;
	controlState = CONTROL_STATE_PRESS;
	bPressed = true;
	ControlEventArgs e(this);
	ofNotifyEvent(press, e);
}

void ControlBase::onRelease(int x, int y, int button){
	if(isDisabled())return;
	bPressed = false;
	if(hitTest(x, y)){
		onRollOver(x, y);
	} else {
		controlState = CONTROL_STATE_IDLE;
	}
	ControlEventArgs e(this);
	ofNotifyEvent(release, e);
}

void ControlBase::onReleaseOutside(int x, int y, int button){
	if(isDisabled())return;
	bPressed = false;
	controlState = CONTROL_STATE_IDLE;
}

void ControlBase::onRollOver(int x, int y){
	if(isDisabled())return;
	controlState = CONTROL_STATE_OVER;
	ControlEventArgs e(this);
	ofNotifyEvent(rollOver, e);
}

void ControlBase::onRollOut(){
	if(isDisabled())return;
	controlState = CONTROL_STATE_IDLE;
	ControlEventArgs e(this);
	ofNotifyEvent(rollOut, e);
}

void ControlBase::onDragOver(int x, int y, int button){
	if(isDisabled())return;
	if(bPressed) controlState = CONTROL_STATE_PRESS;
	else controlState = CONTROL_STATE_OVER;
}

void ControlBase::onDragOutside(int x, int y, int button){
	if(isDisabled())return;
	controlState = CONTROL_STATE_OVER;
}

void ControlBase::drawFill(){
	switch(controlState){
		case CONTROL_STATE_IDLE:
			ofSetColor(colors.mainColorIdle.r, colors.mainColorIdle.g, colors.mainColorIdle.b);
			break;
		case CONTROL_STATE_OVER:
			ofSetColor(colors.mainColorOver.r, colors.mainColorOver.g, colors.mainColorOver.b);
			break;
		case CONTROL_STATE_PRESS:
			ofSetColor(colors.mainColorPress.r, colors.mainColorPress.g, colors.mainColorPress.b);
			break;
	}
	glVertexPointer(2, GL_FLOAT, 0, &vertices[0]);
	glDrawArrays(GL_POLYGON, 0, 4);
}

void ControlBase::drawOutline(){
	glVertexPointer(2, GL_FLOAT, 0, &vertices[0]);
	ofSetColor(colors.outlineColor.r, colors.outlineColor.g, colors.outlineColor.b);
	glDrawArrays(GL_LINE_STRIP, 0, 5);
}

void ControlBase::drawText(){
	ofSetColor(colors.textColor.r, colors.textColor.g, colors.textColor.b);
	ofDrawBitmapString(name, x + textOffSet.x, y + textOffSet.y);
}

void ControlBase::draw(){
	drawFill();
	drawOutline();
	drawText();
}

ControlColorScheme& ControlBase::getColorScheme(){
	return colors;	
}

void ControlBase::setDefaultColors(){
	/*colors.mainColorIdle.r =255;
	colors.mainColorIdle.g = 255;
	colors.mainColorIdle.b = 255;
	
	colors.mainColorOver.r = 222;
	colors.mainColorOver.g = 223;
	colors.mainColorOver.b = 225;
	
	colors.mainColorPress.r = 220;
	colors.mainColorPress.g = 220;
	colors.mainColorPress.b = 220;
	
	colors.outlineColor = colors.mainColorPress;	*/
}
string ControlBase::generateXMLTag(){
	string tag = getName() + "_Type" + ofToString(type, 0) + "_ID" + ofToString(controlId,0);
	for(int i=0; i<tag.size(); i++){
		if(tag[i] == ' ') tag[i] = '_';
	}
	return tag;	
}

bool ControlBase::isPointInside(const ofPoint& pt){
	return pt.x > x && pt.y > y && pt.x < x + width && pt.y < y + height;
}

