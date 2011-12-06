

#include "MyRadioButtons.h"

MyRadioButtons::MyRadioButtons(string _name, float _x, float _y, bool _bAlwaysOn):
MyPanel(_name, _x, _y)
{
	alwaysOn(_bAlwaysOn);
}

MyRadioButtons::~MyRadioButtons(){
	clear();
}

void MyRadioButtons::clear(){
	MyPanel::clear();
	clearToggles();
}

void MyRadioButtons::clearToggles(){
	for(int i=0; i<buttons.size(); i++){
		ofRemoveListener(buttons[i]->turnOn, this, &MyRadioButtons::onToggleTurnOn);
		delete buttons[i];
	}
	buttons.clear();
}

void MyRadioButtons::addToggle(MyToggle* b){
	buttons.push_back(b);	
	ofAddListener(b->turnOn, this, &MyRadioButtons::onToggleTurnOn);
	addControl(b);
	if(alwaysOn() && buttons.size() == 1) buttons[0]->turnOnToggle();
}

vector<MyToggle*>& MyRadioButtons::getRadioButtons(){
	return buttons;	
}

void MyRadioButtons::onToggleTurnOn(ControlEventArgs& e){
	for(int i=0; i<buttons.size(); i++){
		if(buttons[i] == e.control){
			buttons[i]->setState(CONTROL_STATE_PRESS);
		} else {
			if(buttons[i]->getState() == CONTROL_STATE_PRESS){
				buttons[i]->turnOffToggle();
			}
		}
	}
}

void MyRadioButtons::onRollOut(){
	MyPanel::onRollOut();
}

void MyRadioButtons::alwaysOn(bool _allwaysOn){
	bAlwaysOn = _allwaysOn;
}

bool MyRadioButtons::alwaysOn(){
	return bAlwaysOn;	
}

void MyRadioButtons::onPress(int x, int y, int button){
	if(bAlwaysOn){
		for(int i=0; i<buttons.size(); i++){
			if(buttons[i]->getState() == CONTROL_STATE_PRESS){
				if(buttons[i]->hitTest(x, y)) return;
			}
		}
	}
	MyPanel::onPress(x, y, button);
}

void MyRadioButtons::enable(){
	for(int i=0; i<buttons.size(); i++){
		ofAddListener(buttons[i]->turnOn, this, &MyRadioButtons::onToggleTurnOn);
	}
}

void MyRadioButtons::disable(){
	for(int i=0; i<buttons.size(); i++){
		ofRemoveListener(buttons[i]->turnOn, this, &MyRadioButtons::onToggleTurnOn);
	}
}