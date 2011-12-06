#include "MyPanel.h"

MyPanel::MyPanel(string _name, float _x, float _y):
ControlBase(_name)
{
	type= CONTROL_TYPE_PANEL;
	setPos(_x, _y);
	setSize(30, 30);
	adjustVertices();
	setDefaultColors();
}

MyPanel::~MyPanel(){
	clear();
}

void MyPanel::clear(){
	for(int i=0; i<controls.size(); i++){
		delete controls[i];
	}
	controls.clear();
}

void MyPanel::update(){
	for(int i = 0; i<controls.size(); i++){
		controls[i]->update();
	}
}

void MyPanel::draw(){
	ControlBase::drawFill();
	for(int i=0; i<controls.size(); i++){
		controls[i]->draw();
	}
	ControlBase::drawOutline();
	ofSetColor(colors.textColor.r, colors.textColor.g, colors.textColor.b);
	ofDrawBitmapString(name, x + textOffSet.x, y + textOffSet.y);
}

void MyPanel::addControl(ControlBase* c){
	controls.push_back(c);
	c->setPos(c->x + x, c->y + y);
	c->adjustVertices();
	bool bChangeSize = false;
	float newW = width, newH = height;
	if(c->x + c->width > x + width){
		bChangeSize = true;
		newW = c->x + c->width - x + 10;
	};
	
	if(c->y + c->height> y + height){
		bChangeSize = true;
		newH = c->y + c->height - y + 10;
	};
	
	if(bChangeSize){
		setSize(newW, newH);
		adjustVertices();
	}
}

vector<ControlBase*>& MyPanel::getControls(){
	return controls;
}

void MyPanel::setDefaultColors(){
	colors.mainColorOver = colors.mainColorPress = colors.mainColorIdle;
}

void MyPanel::onPress(int x, int y, int button){
	ofMouseEventArgs e;
	e.x = x;
	e.y = y;
	e.button = button;
	for(int i=0; i<controls.size(); i++){
		controls[i]->_mousePressed(e);
	}
}

void MyPanel::onRelease(int x, int y, int button){
	ofMouseEventArgs e;
	e.x = x;
	e.y = y;
	e.button = button;
	for(int i=0; i<controls.size(); i++){
		controls[i]->_mouseReleased(e);
	}
}

void MyPanel::onRollOut(){
	for(int i=0; i<controls.size(); i++){
		if(controls[i]->getState() == CONTROL_STATE_OVER){
			controls[i]->setState(CONTROL_STATE_IDLE);
		}
	}
}

void MyPanel::onReleaseOutside(int x, int y, int button){
	onRelease(x, y, button);
}

void MyPanel::onMouseMove(int x, int y){
	ofMouseEventArgs e;
	e.x = x;
	e.y = y;
	e.button = 0;
	for(int i=0; i<controls.size(); i++){
		controls[i]->_mouseMoved(e);
	}
}

void MyPanel::onDragOver(int x, int y, int button){
	ofMouseEventArgs e;
	e.x = x;
	e.y = y;
	e.button = button;
	for(int i=0; i<controls.size(); i++){
		controls[i]->_mouseDragged(e);
	}
}

void MyPanel::onDragOutside(int x, int y, int button){
	onDragOver(x, y, button);
}

void MyPanel::loadFromXML(ofxXmlSettings& xml){
	for(int i=0; i<controls.size(); i++){
		controls[i]->loadFromXML(xml);
	}
}

void MyPanel::saveToXML(ofxXmlSettings& xml){
	for(int i=0; i<controls.size(); i++){
		controls[i]->saveToXML(xml);
	}
}
