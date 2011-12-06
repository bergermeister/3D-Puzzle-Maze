#include "MyScroller.h"
#include "MyButton.h"

MyScroller::MyScroller(float _x, float _y, float _w, float _h):
ControlBase("")
{
	type = CONTROL_TYPE_SCROLLER;
	setPos(_x, _y);
	setSize(_w, _h);
	adjustVertices();
	setDefaultColors();
	bDrag = false;
	oldScrollerPosY = y;
	positionNorm = 0.0f;
	dragRect.x = x;
	dragRect.y = y;
	dragRect.width = width;
	dragRect.height = 20;
}

MyScroller::~MyScroller(){

}

void MyScroller::update(){
	ControlBase::update();
	if(bDrag){
		dragRect.y = MIN(y + height - dragRect.height, MAX(y, _mouseY - mouseOffSetY));
		positionNorm = (dragRect.y - y) / (height - dragRect.height);
		if(dragRect.y != oldScrollerPosY){
			ControlEventArgs e(this);
			ofNotifyEvent(drag, e, this);
		}
		oldScrollerPosY = dragRect.y;
	}
}

void MyScroller::draw(){
	ControlBase::draw();
	if(bDrag){
		ofSetColor(colors.secColorPress.r, colors.secColorPress.g, colors.secColorPress.b);
	}
	else{
		if(controlState == CONTROL_STATE_OVER){
			ofSetColor(colors.secColorOver.r, colors.secColorOver.g, colors.secColorOver.b);
		} else if(controlState == CONTROL_STATE_IDLE){
			ofSetColor(colors.secColorIdle.r, colors.secColorIdle.g, colors.secColorIdle.b);
		}
	}
	ofRect(dragRect.x, dragRect.y, dragRect.width, dragRect.height);
}

float MyScroller::getPosition(){
	return positionNorm;	
}

void MyScroller::onPress(int _x, int _y, int _button){
	ControlBase::onPress(_x, _y, _button);
	if(_y > dragRect.y && _y < dragRect.y + dragRect.height){
		mouseOffSetY = _y - dragRect.y;
	} else mouseOffSetY = dragRect.height * 0.5f;
	bDrag = true;
}

void MyScroller::onRelease(int _x, int _y, int _button){
	ControlBase::onRelease(_x, _y, _button);
	bDrag = false;
}

void MyScroller::onReleaseOutside(int _x, int _y, int _button){
	onRelease(_x, _y, _button);
}

void MyScroller::setDefaultColors(){
	colors.mainColorOver = colors.mainColorPress = colors.mainColorIdle;
}

void MyScroller::adjustVertices(){
	ControlBase::adjustVertices();
	dragRect.x = x;
	dragRect.y = y + (positionNorm * (height - 20)); 
	dragRect.width = width;
	dragRect.height = 20;
}

void MyScroller::setPosition(float _posNorm){
	positionNorm = MIN(1, MAX(0, _posNorm));
	dragRect.y = y + ((height - dragRect.height)*positionNorm);
}





