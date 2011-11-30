
#include "MyGUI.h"

MyGUI::MyGUI(){
	bEnabled = false;
	enable();
	glEnableClientState(GL_VERTEX_ARRAY);
	setXMLFile();
}

MyGUI::~MyGUI(){
	for(int i=0; i<controls.size(); i++){
		delete controls[i];
	}
	controls.clear();
}

void MyGUI::addControl(ControlBase* control){
	controls.push_back(control);
}
					

void MyGUI::draw(){
	if(!bEnabled)return;
	for(int i=0; i<controls.size(); i++){
		ControlBase& control = *controls[i];
		if(control.isEnabled()){
			control.draw();
		}
	}
}

void MyGUI::enable(){
	if(isEnabled())return;
	ofAddListener(ofEvents.update, this, &MyGUI::update);
	ofAddListener(ofEvents.mousePressed, this, &MyGUI::mousePressed);
	ofAddListener(ofEvents.mouseMoved, this, &MyGUI::mouseMoved);
	ofAddListener(ofEvents.mouseDragged, this, &MyGUI::mouseDragged);
	ofAddListener(ofEvents.mouseReleased, this, &MyGUI::mouseReleased);
	ofAddListener(ofEvents.keyPressed, this, &MyGUI::keyPressed);
	ofAddListener(ofEvents.keyReleased, this, &MyGUI::keyReleased);
	bEnabled = true;
}

void MyGUI::disable(){
	if(isDisabled()) return;
	ofRemoveListener(ofEvents.update, this, &MyGUI::update);
	ofRemoveListener(ofEvents.mousePressed, this, &MyGUI::mousePressed);
	ofRemoveListener(ofEvents.mouseMoved, this, &MyGUI::mouseMoved);
	ofRemoveListener(ofEvents.mouseDragged, this, &MyGUI::mouseDragged);
	ofRemoveListener(ofEvents.mouseReleased, this, &MyGUI::mouseReleased);
	ofRemoveListener(ofEvents.keyPressed, this, &MyGUI::keyPressed);
	ofRemoveListener(ofEvents.keyReleased, this, &MyGUI::keyReleased);
	bEnabled = false;
}

void MyGUI::toggle(){
	if(isEnabled()) disable();
	else enable();
}

bool MyGUI::isEnabled(){
	return bEnabled;
}

bool MyGUI::isDisabled(){
	return !bEnabled;
}


void MyGUI::update(ofEventArgs &e){
	if(!bEnabled)return;
	for(int i=0; i<controls.size(); i++){
		ControlBase& control = *controls[i];
		if(control.isEnabled()){
			control.update();
		}
	}
}

void MyGUI::mouseMoved(ofMouseEventArgs &e){
	for(int i=0; i<controls.size(); i++){
		ControlBase& control = *controls[i];
		if(control.isEnabled()){
			control._mouseMoved(e);
		}
	}
}

void MyGUI::mousePressed(ofMouseEventArgs &e){
	for(int i=0; i<controls.size(); i++){
		ControlBase& control = *controls[i];
		if(control.isEnabled()){
			control._mousePressed(e);
		}
	}
}

void MyGUI::mouseDragged(ofMouseEventArgs &e){
	for(int i=0; i<controls.size(); i++){
		ControlBase& control = *controls[i];
		if(control.isEnabled()){
			control._mouseDragged(e);
		}
	}
}

void MyGUI::mouseReleased(ofMouseEventArgs &e){
	for(int i=0; i<controls.size(); i++){
		ControlBase& control = *controls[i];
		if(control.isEnabled()){
			control._mouseReleased(e);
		}
	}
}

void MyGUI::keyPressed(ofKeyEventArgs &e){
	for(int i=0; i<controls.size(); i++){
		ControlBase& control = *controls[i];
		if(control.isEnabled()){
			control._keyPressed(e);
		}
	}
}

void MyGUI::keyReleased(ofKeyEventArgs &e){
	for(int i=0; i<controls.size(); i++){
		ControlBase& control = *controls[i];
		if(control.isEnabled()){
			control._keyReleased(e);
		}
	}
}

void MyGUI::setXMLFile(string _file){
	xmlFile = _file;
}

void MyGUI::loadFromXML(){
	xml.loadFile(xmlFile);
	for(int i=0; i<controls.size(); i++){
		controls[i]->loadFromXML(xml);
	}
}

void MyGUI::saveToXML(){
	xml.clear();
	for(int i=0; i<controls.size(); i++){
		controls[i]->saveToXML(xml);
	}
	xml.saveFile(xmlFile);
}

bool MyGUI::isPointInControl(const ofPoint& pt){
	for(vector<ControlBase*>::iterator it = controls.begin(); it != controls.end(); ++it){
		if((*it)->isPointInside(pt))return true;
	}
	return false;
}

